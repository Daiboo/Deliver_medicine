#ifndef __NI2C_H
#define __NI2C_H
#include <stdint.h>

void I2C0_Init(void);
void i2c0Write(uint8_t addr, uint8_t regAddr, uint8_t data) ;
uint8_t i2c0Read(uint8_t addr, uint8_t regAddr);
void i2c0ReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length);


void I2C1_Init(void);
void i2c1Write(uint8_t addr, uint8_t regAddr, uint8_t data) ;
uint8_t i2c1Read(uint8_t addr, uint8_t regAddr);
void i2c1ReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length);

void Single_WriteI2C(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address);
short int Double_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address);




#endif
