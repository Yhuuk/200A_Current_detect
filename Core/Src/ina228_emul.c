#include "ina228_emul.h"
#include "gpio.h"      // 里面有 LED/RGB 等定义，这里主要为了拿到 GPIO_TypeDef
#include "i2c.h"

// ---------- INA228 常量（和 PX4 驱动保持一致） ----------
#define INA228_I2C_ADDRESS      0x45      // 7-bit 地址（和 PX4 INA228_BASEADDR 一样）

#define INA228_REG_CONFIG       0x00
#define INA228_REG_ADC_CONFIG   0x01
#define INA228_REG_SHUNT_CAL    0x02
#define INA228_REG_VSHUNT       0x04
#define INA228_REG_VBUS         0x05
#define INA228_REG_DIETEMP      0x06
#define INA228_REG_CURRENT      0x07
#define INA228_REG_POWER        0x08
#define INA228_REG_MANUF_ID     0x3E
#define INA228_REG_DEVICE_ID    0x3F

#define INA228_MFG_ID_TI        0x5449    // 'TI'
#define INA228_MFG_DIE          0x228

// 电压 LSB（和 PX4 驱动里的 INA228_VSCALE 一致）
#define INA228_VSCALE           1.953125e-4f   // 195.3125 uV/LSB

#define DN_MAX_20BIT            524288.0f      // 2^19

// 这里定义一个“工程可测最大电流”，必须和 PX4 参数 INA228_CURRENT 一致，
// 否则 PX4 解码出来的电流会有比例误差。
#define EMU_MAX_CURRENT_A       200.0f        // 举例：你希望量程是 ±200A

// 对应的 current_lsb
#define EMU_CURRENT_LSB         (EMU_MAX_CURRENT_A / DN_MAX_20BIT)

// // ---------- I2C 句柄 ----------
// // 暂时只在本文件内使用
// static I2C_HandleTypeDef hi2c1;

// ---------- INA228 寄存器镜像 ----------
static uint16_t reg_config      = 0;
static uint16_t reg_adc_config  = 0;
static uint16_t reg_shunt_cal   = 0;

// 20bit 原始码，存放在 int32 里
static int32_t reg_vbus_raw     = 0;  // 总线电压 raw
static int32_t reg_current_raw  = 0;  // 电流 raw

static uint16_t reg_manuf_id    = INA228_MFG_ID_TI;
static uint16_t reg_device_id   = (INA228_MFG_DIE << 4); // 这样 DEVICEID(value) = 0x228

static uint8_t current_reg_addr = 0;  // 最近一次主机写入的寄存器地址

// I2C 中断缓冲
static uint8_t i2c_rx_buf[3];
static uint8_t i2c_tx_buf[3];

// ---------- 工具函数：把 20bit raw 编码成 3 字节（和真实 INA228 一样）----------
// 20bit 补码 raw20 放在 bits [23:4]，低 4 bit 为 0，按大端顺序输出 3 字节
static void encode_20bit_to_3bytes(int32_t raw20, uint8_t out[3])
{
    // 保证 20bit 范围
    if (raw20 >  0x7FFFF) raw20 =  0x7FFFF;
    if (raw20 < -0x80000) raw20 = -0x80000;

    uint32_t raw_u = ((uint32_t)raw20) & 0xFFFFF;
    uint32_t raw24 = raw_u << 4;

    out[0] = (uint8_t)((raw24 >> 16) & 0xFF);
    out[1] = (uint8_t)((raw24 >> 8)  & 0xFF);
    out[2] = (uint8_t)( raw24        & 0xFF);
}

// 根据当前寄存器地址，准备要发给 PX4 的数据
static uint8_t prepare_tx_buffer(uint8_t reg, uint8_t *buf)
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
        // 未实现的寄存器就返回 0
        buf[0] = buf[1] = buf[2] = 0;
        return 2;
    }
}

// // ---------- HAL 的 MSP 初始化（配置 I2C1 时钟和 GPIO） ----------
// // 注意：I2C1 使用 PB6 = SCL, PB7 = SDA
// void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
// {
//     if (hi2c->Instance == I2C1) {
//         GPIO_InitTypeDef GPIO_InitStruct = {0};

//         // GPIOB & I2C1 时钟
//         __HAL_RCC_GPIOB_CLK_ENABLE();
//         __HAL_RCC_I2C1_CLK_ENABLE();

//         // PB6/PB7 复用开漏
//         GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
//         GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//         GPIO_InitStruct.Pull = GPIO_NOPULL;
//         GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//         HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//         // I2C1 事件 & 错误中断（可选，强烈建议打开）
//         HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1, 0);
//         HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

//         HAL_NVIC_SetPriority(I2C1_ER_IRQn, 1, 1);
//         HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
//     }
// }

// ---------- 对外接口：初始化 I2C + 寄存器 ----------
void INA228_Emu_Init(void)
{
    // ? 这里假设 main.c 里已经调用过 MX_I2C1_Init();

    // 初始化固定寄存器
    reg_config    = 0;
    reg_adc_config= 0;
    reg_shunt_cal = 0;

    reg_manuf_id  = INA228_MFG_ID_TI;
    reg_device_id = (INA228_MFG_DIE << 4);

    current_reg_addr = 0;

    // 打开从机监听模式（如果你的 F1 HAL 里有这个 API）
    HAL_I2C_EnableListen_IT(&hi2c1);
}


// ---------- 对外接口：更新测量值 ----------
void INA228_Emu_UpdateMeasurements(float v_bus, float current)
{
    // 总线电压 raw = V / VSCALE
    float raw_v = v_bus / INA228_VSCALE;
    int32_t raw_v_i = (int32_t)(raw_v + (raw_v >= 0 ? 0.5f : -0.5f));

    if (raw_v_i < 0)        raw_v_i = 0;
    if (raw_v_i > 0x7FFFF)  raw_v_i = 0x7FFFF;
    reg_vbus_raw = raw_v_i;

    // 电流 raw = current / current_lsb
    float raw_i = current / EMU_CURRENT_LSB;
    int32_t raw_i_i = (int32_t)(raw_i + (raw_i >= 0 ? 0.5f : -0.5f));

    if (raw_i_i >  0x7FFFF) raw_i_i =  0x7FFFF;
    if (raw_i_i < -0x80000) raw_i_i = -0x80000;
    reg_current_raw = raw_i_i;
}

// ---------- I2C 从机回调（地址匹配） ----------
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c,
                          uint8_t TransferDirection,
                          uint16_t AddrMatchCode)
{
    if (hi2c->Instance != I2C1) {
        return;
    }

    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        // 主机要写数据到我们这里：我们先收最多 3 字节（寄存器地址 + 2 字节数据）
        HAL_I2C_Slave_Receive_IT(hi2c, i2c_rx_buf, sizeof(i2c_rx_buf));

    } else {
        // 主机要读：根据 current_reg_addr 准备数据
        uint8_t len = prepare_tx_buffer(current_reg_addr, i2c_tx_buf);
        HAL_I2C_Slave_Transmit_IT(hi2c, i2c_tx_buf, len);
    }
}

// ---------- 接收完成回调 ----------
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) {
        return;
    }

    // 第一个字节永远是寄存器地址
    current_reg_addr = i2c_rx_buf[0];

    // 如果有多余的两个字节，就当成寄存器写入处理（CONFIG、SHUNTCAL 等）
    uint16_t value = ((uint16_t)i2c_rx_buf[1] << 8) | i2c_rx_buf[2];

    switch (current_reg_addr) {
    case INA228_REG_CONFIG:
        reg_config = value;
        break;

    case INA228_REG_ADC_CONFIG:
        reg_adc_config = value;
        break;

    case INA228_REG_SHUNT_CAL:
        reg_shunt_cal = value;
        break;

    default:
        break;
    }

    // 重新进入监听模式
    HAL_I2C_EnableListen_IT(hi2c);
}

// ---------- 发送完成回调 ----------
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) {
        return;
    }

    HAL_I2C_EnableListen_IT(hi2c);
}

// ---------- STOP 条件回调 ----------
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) {
        return;
    }

    HAL_I2C_EnableListen_IT(hi2c);
}
