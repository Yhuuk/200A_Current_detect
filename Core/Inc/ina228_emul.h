#ifndef INA228_EMUL_H
#define INA228_EMUL_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 I2C1 从机 + INA228 寄存器仿真
void INA228_Emu_Init(void);

// 更新测量值（你后面用 ADC 算出电压、电流后，每隔一段时间调用一次）
// v_bus 单位：V（总线电压）
// current 单位：A（电流，可正可负）
void INA228_Emu_UpdateMeasurements(float v_bus, float current);

#ifdef __cplusplus
}
#endif

#endif // INA228_EMUL_H
