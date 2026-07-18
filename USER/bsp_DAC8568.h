#ifndef __BSP_DAC8568_H
#define __BSP_DAC8568_H

#include "main.h"
#include <stdint.h>


/* GPIO操作宏 */
#define DAC8568_SYNC_H    HAL_GPIO_WritePin(DACSYNC_GPIO_Port, DACSYNC_Pin, GPIO_PIN_SET)
#define DAC8568_SYNC_L    HAL_GPIO_WritePin(DACSYNC_GPIO_Port, DACSYNC_Pin, GPIO_PIN_RESET)

#define DAC8568_SCLK_H    HAL_GPIO_WritePin(GPIOE, DACSCLK_Pin, GPIO_PIN_SET)
#define DAC8568_SCLK_L    HAL_GPIO_WritePin(GPIOE, DACSCLK_Pin, GPIO_PIN_RESET)

#define DAC8568_DIN_H     HAL_GPIO_WritePin(GPIOE, DACDIN_Pin, GPIO_PIN_SET)
#define DAC8568_DIN_L     HAL_GPIO_WritePin(GPIOE, DACDIN_Pin, GPIO_PIN_RESET)




/************************DA8568寄存器SR值*****************************/
#define PrefixControlbyte                   0x03
#define AddressOutA                         0x0
#define AddressOutB                         0x1
#define AddressOutC                         0x2
#define AddressOutD                         0x3
#define AddressOutE                         0x4
#define AddressOutF                         0x5
#define AddressOutG                         0x6
#define AddressOutH                         0x7
#define Featurebyte                         0x0

/*****DA8568寄存器SR值-结束*************************************/

///****************************DAC8568命令************************/
#define SETUP_INTERNAL_REGISTER            0
#define POWER_UP                           1
#define DAC8568RESET                       2
///*****DAC8568命令-结束*******************************************/

/* 通道快捷定义 */
#define OutA    AddressOutA
#define OutB    AddressOutB
#define OutC    AddressOutC
#define OutD    AddressOutD
#define OutE    AddressOutE
#define OutF    AddressOutF
#define OutG    AddressOutG
#define OutH    AddressOutH

#define DAC8568_CODE_MAX         (65535U)
#define DAC8568_CODE_COUNT       (65536.0)

#define DAC8568_FULL_SCALE_V     (5.0)
#define DAC8568_FULL_SCALE_MV    (5000L)


void DAC8568_Init(void);
void SPI_SendHalfWord(uint16_t m);
void SPI_SendByte(uint8_t m);
void DAC8568_SetVoltage(uint8_t channel, double voltage_v);
void DAC8568_SetDAC(uint8_t channel, int32_t dac_set);
double DAC8568_CodeToVoltage(uint16_t code);

extern volatile uint16_t g_dac8568_code[8];




#endif
