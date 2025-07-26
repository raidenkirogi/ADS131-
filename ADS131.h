/**
 ******************************************************************************
 * 配置:
 * - MCU: STM32H750
 * - SPI: SPI4
 * - CS Pin: PE4
 * - DRDY Pin: PE3 (外部中断)
 * - RESET Pin: PC13
 * - Word Length: 24-bit
 * - CRC: Disabled
 * - Mode: Asynchronous Slave Mode (M0=1, M1=0, M2=0)
 ******************************************************************************
 */

#ifndef ADS131_H_
#define ADS131_H_

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
 
//==============================================================================
// 1. 用户硬件配置
//==============================================================================

// 定义使用的 SPI 句柄 (要在 main.c 中定义并初始化 hspi4)
extern SPI_HandleTypeDef hspi4;
#define ADS131_SPI_HANDLE (&hspi4)

// CS (片选) 引脚
#define ADS131_CS_GPIO_Port     GPIOE
#define ADS131_CS_Pin           GPIO_PIN_4

// nRESET (复位) 引脚
#define ADS131_RESET_GPIO_Port  GPIOC
#define ADS131_RESET_Pin        GPIO_PIN_13

// nDRDY (数据就绪) 引脚 - 外部中断 EXTI
#define ADS131_DRDY_GPIO_Port   GPIOE
#define ADS131_DRDY_Pin         GPIO_PIN_3

//==============================================================================
// 2. ADC 驱动配置
//==============================================================================

// SPI 帧模式 - 推荐使用固定模式 (FIXED mode)
#define SET_FIXED

// CRC 校验 - 禁用
// #define SET_CRC_EN

// M 引脚硬件配置决定了 ADC 的工作模式
// M0=1 -> 异步从机模式
#define ASYNC_SLAVE_MODE

// M1=0 -> 24位数据长度
#define WORD_LENGTH_BITS        ((uint8_t) 24)
#define WORD_LENGTH_24BIT

// M2=0 -> Hamming 码校验关闭
// #define HAMMING_ENABLED

//==============================================================================
// 3. 数据结构和全局变量声明
//==============================================================================
 
/**
 * @brief ADC 采样数据结构体
 */
typedef struct
{
    int32_t channel1;   // 通道 1 数据
    int32_t channel2;   // 通道 2 数据
    int32_t channel3;   // 通道 3 数据
    int32_t channel4;   // 通道 4 数据
    uint16_t response;  // 状态响应字
} ads131_data_t;
 
// 数据就绪标志位 (在 EXTI 中断服务程序中设置为 true)
extern volatile bool g_ads131_data_ready;

//==============================================================================
// 4. 函数原型声明
//==============================================================================

// --- 主要功能函数 ---
bool     ADS131_Init(void);
bool     ADS131_Startup(void);
bool     ADS131_ReadData(ads131_data_t *dataStruct);
uint8_t  ADS131_ReadSingleRegister(uint8_t address);
void     ADS131_WriteSingleRegister(uint8_t address, uint8_t data);
uint16_t ADS131_SendCommand(uint16_t opcode);
bool     ADS131_LockRegisters(void);
bool     ADS131_UnlockRegisters(void);

// --- 辅助函数 ---
void     ADS131_Reset(void);
uint8_t  ADS131_GetRegisterValue(uint8_t address);
void     ADS131_RestoreRegisterDefaults(void);
int32_t  ADS131_SignExtend(const uint8_t dataBytes[]);

//==============================================================================
// 5. ADS131 命令和寄存器定义
//==============================================================================

// --- Device commands ---
#define OPCODE_NULL                             ((uint16_t) 0x0000)
#define OPCODE_RESET                            ((uint16_t) 0x0011)
#define OPCODE_STANDBY                          ((uint16_t) 0x0022)
#define OPCODE_WAKEUP                           ((uint16_t) 0x0033)
#define OPCODE_LOCK                             ((uint16_t) 0x0555)
#define OPCODE_UNLOCK                           ((uint16_t) 0x0655)
#define OPCODE_RREG                             ((uint16_t) 0x2000)
#define OPCODE_WREG                             ((uint16_t) 0x4000)

// --- Register definitions ---
#define NUM_REGISTERS           ((uint8_t) 21)

#define ID_MSB_ADDRESS          ((uint8_t) 0x00)
#define ID_LSB_ADDRESS          ((uint8_t) 0x01)
#define STAT_1_ADDRESS          ((uint8_t) 0x02)
#define STAT_P_ADDRESS          ((uint8_t) 0x03)
#define STAT_N_ADDRESS          ((uint8_t) 0x04)
#define STAT_S_ADDRESS          ((uint8_t) 0x05)
#define ERROR_CNT_ADDRESS       ((uint8_t) 0x06)
#define STAT_M2_ADDRESS         ((uint8_t) 0x07)
#define A_SYS_CFG_ADDRESS       ((uint8_t) 0x0B)
#define D_SYS_CFG_ADDRESS       ((uint8_t) 0x0C)
#define CLK1_ADDRESS            ((uint8_t) 0x0D)
#define CLK2_ADDRESS            ((uint8_t) 0x0E)
#define ADC_ENA_ADDRESS         ((uint8_t) 0x0F)
#define ADC1_ADDRESS            ((uint8_t) 0x11)
#define ADC2_ADDRESS            ((uint8_t) 0x12)
#define ADC3_ADDRESS            ((uint8_t) 0x13)
#define ADC4_ADDRESS            ((uint8_t) 0x14)

// --- Register default values and masks ---
#define ID_MSB_NU_CH_4          ((uint8_t) 0x04)
#define STAT_1_DEFAULT          ((uint8_t) 0x00)
#define STAT_P_DEFAULT          ((uint8_t) 0x00)
#define STAT_N_DEFAULT          ((uint8_t) 0x00)
#define STAT_S_DEFAULT          ((uint8_t) 0x00)
#define ERROR_CNT_DEFAULT       ((uint8_t) 0x00)
#define STAT_M2_DEFAULT         ((uint8_t) 0x00)
#define STAT_M2_DEFAULT_MASK    ((uint8_t) 0xC0)
#define A_SYS_CFG_DEFAULT       ((uint8_t) 0x60)
#define D_SYS_CFG_DEFAULT       ((uint8_t) 0x3C)
#define CLK1_DEFAULT            ((uint8_t) 0x08)
#define CLK2_DEFAULT            ((uint8_t) 0x86)
#define ADC_ENA_DEFAULT         ((uint8_t) 0x00)
#define ADC1_DEFAULT            ((uint8_t) 0x00)
#define ADC2_DEFAULT            ((uint8_t) 0x00)
#define ADC3_DEFAULT            ((uint8_t) 0x00)
#define ADC4_DEFAULT            ((uint8_t) 0x00)

#define D_SYS_CFG_FIXED_MASK    ((uint8_t) 0x02)
#define D_SYS_CFG_CRC_EN_MASK   ((uint8_t) 0x01)
#define CLK1_CLK_DIV_2          ((uint8_t) 0x02)
#define CLK2_ICLK_DIV_2         ((uint8_t) 0x20)
#define CLK2_OSR_4096           ((uint8_t) 0x00)
#define CLK2_OSR_2048           ((uint8_t) 0x01)
#define CLK2_OSR_1024           ((uint8_t) 0x02)
#define CLK2_OSR_800            ((uint8_t) 0x03)
#define CLK2_OSR_768            ((uint8_t) 0x04)
#define CLK2_OSR_512            ((uint8_t) 0x05)
#define CLK2_OSR_400            ((uint8_t) 0x06)
#define CLK2_OSR_384            ((uint8_t) 0x07)
#define CLK2_OSR_256            ((uint8_t) 0x08)
#define CLK2_OSR_200            ((uint8_t) 0x09)
#define CLK2_OSR_192            ((uint8_t) 0x0A)
#define CLK2_OSR_128            ((uint8_t) 0x0B)
#define CLK2_OSR_96             ((uint8_t) 0x0C)
#define CLK2_OSR_64             ((uint8_t) 0x0D)
#define CLK2_OSR_48             ((uint8_t) 0x0E)
#define CLK2_OSR_32             ((uint8_t) 0x0F)
#define ADC_ENA_ENA_ALL_CH_PWUP ((uint8_t) 0x0F)

#endif /* ADS131_H_ */
