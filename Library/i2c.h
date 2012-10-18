#ifndef I2C_H
#define I2C_H

#include <stm32f10x_i2c.h>

typedef enum {Error = 0, Success = !Error } Status;

Status I2C_Read(I2C_TypeDef* I2Cx, uint8_t* buf, uint32_t nbuf, uint8_t SlaveAddress);
Status I2C_Write(I2C_TypeDef* I2Cx, const uint8_t* buf, uint32_t nbuf,  uint8_t SlaveAddress);
void I2C_LowLevel_Init(I2C_TypeDef* I2Cx, int ClockSpeed, int OwnAddress);

#endif
