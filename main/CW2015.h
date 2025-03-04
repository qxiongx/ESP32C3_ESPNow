#ifndef CW2015_H
#define CW2015_H

#include "driver/i2c.h"

#define CW_ADDR (0xC4 >> 1)// CW2015的I2C地址，高7位
#define CW_VERSION 0x00 // 版本号寄存器地址
#define CW_VCELL_H 0x02 // 电压寄存器地址
#define CW_VCELL_L 0x03
#define CW_SOC_integer 0x04 // 电量整数部分寄存器地址
#define CW_SOC_decimal 0x05 // 电量小数部分寄存器地址
#define CW_RRT_H  0x06 //剩余时间显示（未使用）
#define CW_RRT_L  0x07
#define CONFIG 0x08  //低电量阈值
#define CW_MODE 0x0A
#define CW_MODE_SLEEP 0xC0
#define CW_MODE_NORMAL 0x00
#define CW_MODE_QUICK_START 0x30
#define CW_MODE_RESTART 0x0F


void IRAM_ATTR timer_isr(void *arg);
void initCW2015(void);
uint16_t readAnalogVoltage(void);
uint8_t readBatQuantityLowPre(void);
float readBatQuantityHighPre(void);
float readBatVoltage(void);
void checkCW2015(void);
void setSleep(void);
void wakeUp(void);
void quickStart(void);
void powerReset(void);
uint8_t readRegister(uint8_t reg);
void writeRegister(uint8_t val, uint8_t reg);
uint8_t readCW2015Version(void);
uint16_t readRemainingRunTime(void);
void uart_event_task(void *pvParameters);
esp_err_t i2c_master_init(void);
#endif