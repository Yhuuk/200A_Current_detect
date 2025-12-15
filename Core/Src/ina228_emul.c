#include "ina228_emul.h"
#include "i2c.h"

// ---------- INA228 常量 ----------
#define INA228_I2C_ADDRESS      0x45

#define INA228_REG_CONFIG       0x00
#define INA228_REG_ADC_CONFIG   0x01
#define INA228_REG_SHUNT_CAL    0x02
#define INA228_REG_VBUS         0x05
#define INA228_REG_CURRENT      0x07
#define INA228_REG_MANUF_ID     0x3E
#define INA228_REG_DEVICE_ID    0x3F

#define INA228_MFG_ID_TI        0x5449
#define INA228_MFG_DIE          0x228

#define INA228_VSCALE           1.953125e-4f
#define DN_MAX_20BIT            524288.0f
#define EMU_MAX_CURRENT_A       200.0f
#define EMU_CURRENT_LSB         (EMU_MAX_CURRENT_A / DN_MAX_20BIT)

// ---------- 寄存器镜像 ----------
static volatile uint16_t reg_config     = 0;
static volatile uint16_t reg_adc_config = 0;
static volatile uint16_t reg_shunt_cal  = 0;

static volatile int32_t  reg_vbus_raw    = 0;
static volatile int32_t  reg_current_raw = 0;

static volatile uint16_t reg_manuf_id  = INA228_MFG_ID_TI;
static volatile uint16_t reg_device_id = (INA228_MFG_DIE << 4);

static volatile uint8_t current_reg_addr = 0;

// I2C 收发状态
static volatile uint8_t rx_state = 0;   // 0:等寄存器地址  1:等MSB  2:等LSB
static volatile uint8_t rx_msb   = 0;

static volatile uint8_t tx_buf[3];
static volatile uint8_t tx_len = 0;
static volatile uint8_t tx_idx = 0;

// ---------- 工具：20bit -> 3字节（大端，低4bit=0） ----------
static void encode_20bit_to_3bytes(int32_t raw20, uint8_t out[3])
{
    if (raw20 >  0x7FFFF) raw20 =  0x7FFFF;
    if (raw20 < -0x80000) raw20 = -0x80000;

    uint32_t raw_u  = ((uint32_t)raw20) & 0xFFFFF;
    uint32_t raw24  = raw_u << 4;

    out[0] = (uint8_t)((raw24 >> 16) & 0xFF);
    out[1] = (uint8_t)((raw24 >> 8)  & 0xFF);
    out[2] = (uint8_t)( raw24        & 0xFF);
}

static uint8_t prepare_tx(uint8_t reg, uint8_t *buf)
{
    switch (reg) {
    case INA228_REG_CONFIG:
        buf[0] = (uint8_t)(reg_config >> 8);
        buf[1] = (uint8_t)(reg_config & 0xFF);
        return 2;

    case INA228_REG_ADC_CONFIG:
        buf[0] = (uint8_t)(reg_adc_config >> 8);
        buf[1] = (uint8_t)(reg_adc_config & 0xFF);
        return 2;

    case INA228_REG_SHUNT_CAL:
        buf[0] = (uint8_t)(reg_shunt_cal >> 8);
        buf[1] = (uint8_t)(reg_shunt_cal & 0xFF);
        return 2;

    case INA228_REG_VBUS:
        encode_20bit_to_3bytes(reg_vbus_raw, buf);
        return 3;

    case INA228_REG_CURRENT:
        encode_20bit_to_3bytes(reg_current_raw, buf);
        return 3;

    case INA228_REG_MANUF_ID:
        buf[0] = (uint8_t)(reg_manuf_id >> 8);
        buf[1] = (uint8_t)(reg_manuf_id & 0xFF);
        return 2;

    case INA228_REG_DEVICE_ID:
        buf[0] = (uint8_t)(reg_device_id >> 8);
        buf[1] = (uint8_t)(reg_device_id & 0xFF);
        return 2;

    default:
        buf[0] = buf[1] = buf[2] = 0;
        return 2;
    }
}

void INA228_Emu_Init(void)
{
    // 固定寄存器
    reg_manuf_id  = INA228_MFG_ID_TI;
    reg_device_id = (INA228_MFG_DIE << 4);

    // 开 ACK + 开中断（注意 OR，别覆盖 CR2 里的频率配置）
    hi2c1.Instance->CR1 |= I2C_CR1_ACK;
    hi2c1.Instance->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;

    // 清一下状态
    rx_state = 0;
    tx_len = tx_idx = 0;
}

void INA228_Emu_UpdateMeasurements(float v_bus, float current)
{
    // VBUS raw
    float raw_v = v_bus / INA228_VSCALE;
    int32_t rv = (int32_t)(raw_v + 0.5f);
    if (rv < 0) rv = 0;
    if (rv > 0x7FFFF) rv = 0x7FFFF;
    reg_vbus_raw = rv;

    // CURRENT raw
    float raw_i = current / EMU_CURRENT_LSB;
    int32_t ri = (int32_t)(raw_i + (raw_i >= 0 ? 0.5f : -0.5f));
    if (ri >  0x7FFFF) ri =  0x7FFFF;
    if (ri < -0x80000) ri = -0x80000;
    reg_current_raw = ri;
}

// --------- I2C1 事件中断：直接按 SR1/SR2 做从机状态机 ----------
void INA228_Emu_I2C1_EV_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;

    // 1) 地址匹配
    if (sr1 & I2C_SR1_ADDR) {
        volatile uint32_t tmp = I2C1->SR2; (void)tmp; // 读 SR2 清 ADDR

        // 主机读：从机发
        if (I2C1->SR2 & I2C_SR2_TRA) {
            tx_len = prepare_tx(current_reg_addr, (uint8_t*)tx_buf);
            tx_idx = 0;

            if (I2C1->SR1 & I2C_SR1_TXE) {
                I2C1->DR = tx_buf[tx_idx++];
            }
        } else {
            // 主机写：从机收
            rx_state = 0;   // 等寄存器地址
        }
    }

    // 2) 接收字节
    if (sr1 & I2C_SR1_RXNE) {
        uint8_t b = (uint8_t)I2C1->DR;

        if (rx_state == 0) {
            // 第1字节：寄存器指针
            current_reg_addr = b;
            rx_state = 1; // 可能还有 MSB/LSB（写寄存器）
        } else if (rx_state == 1) {
            // 第2字节：MSB
            rx_msb = b;
            rx_state = 2;
        } else {
            // 第3字节：LSB -> 写入 16bit 寄存器
            uint16_t v = ((uint16_t)rx_msb << 8) | b;

            switch (current_reg_addr) {
            case INA228_REG_CONFIG:     reg_config     = v; break;
            case INA228_REG_ADC_CONFIG: reg_adc_config = v; break;
            case INA228_REG_SHUNT_CAL:  reg_shunt_cal  = v; break;
            default: break;
            }

            // 写完一笔，回到等指针
            rx_state = 0;
        }
    }

    // 3) 发送字节
    if (sr1 & I2C_SR1_TXE) {
        if (tx_idx < tx_len) {
            I2C1->DR = tx_buf[tx_idx++];
        } else {
            I2C1->DR = 0x00;
        }
    }

    // 4) STOP
    if (sr1 & I2C_SR1_STOPF) {
        volatile uint32_t tmp = I2C1->SR1; (void)tmp;
        I2C1->CR1 |= I2C_CR1_ACK; // 写 CR1 清 STOPF
        rx_state = 0;
        tx_len = tx_idx = 0;
    }
}

// --------- I2C1 错误中断：清标志 ----------
void INA228_Emu_I2C1_ER_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;

    if (sr1 & I2C_SR1_AF) {
        I2C1->SR1 &= ~I2C_SR1_AF; // master NACK（正常结束读）
        tx_len = tx_idx = 0;
    }
    if (sr1 & I2C_SR1_BERR) I2C1->SR1 &= ~I2C_SR1_BERR;
    if (sr1 & I2C_SR1_ARLO) I2C1->SR1 &= ~I2C_SR1_ARLO;
    if (sr1 & I2C_SR1_OVR)  I2C1->SR1 &= ~I2C_SR1_OVR;
}
