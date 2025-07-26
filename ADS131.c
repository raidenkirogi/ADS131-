ADS131驱动代码
#include "ads131.h"

//==============================================================================
// 内部宏定义
//==============================================================================

// 根据数据位宽定义每个字(Word)的字节数
#define WORD_LENGTH_BYTES   (WORD_LENGTH_BITS >> 3)

// 从 16 位数据中提取高/低字节
#define UPPER_BYTE(x)       ((uint8_t) ((0xFF00 & x) >> 8))
#define LOWER_BYTE(x)       ((uint8_t)  (0x00FF & x))

// 合并两个 8 位数据为一个 16 位数据
#define COMBINE_BYTES(x, y) ((((uint16_t) x) << 8) | ((uint16_t) y))

// SPI 通信超时时间 (ms)
#define SPI_TIMEOUT         100

//==============================================================================
// 内部静态变量
//==============================================================================

// 存储默认 ADC 寄存器值
static uint8_t registerMap[NUM_REGISTERS];

// 寄存器锁定状态标志 (设备上电后默认为锁定状态)
static bool registersLocked = true;

// SPI 发送和接收缓冲区
// 帧结构: [响应字] + [通道1] + [通道2] + [通道3] + [通道4] + [CRC字]
// 总共 6 个字
static uint8_t spi_tx_data[6 * WORD_LENGTH_BYTES] = {0};
static uint8_t spi_rx_data[6 * WORD_LENGTH_BYTES] = {0};

// SPI 帧中的字节总数
#ifdef SET_FIXED
    // 固定模式下，帧长度固定为 6 个字
    static const uint8_t spi_frame_byte_len = 6 * WORD_LENGTH_BYTES;
#else
    // 动态模式下，帧长度可变 (没写完)
    static const uint8_t spi_frame_byte_len = 1 * WORD_LENGTH_BYTES;
#endif
 
 //==============================================================================
 // 底层硬件抽象函数 (HAL Abstraction)
 //==============================================================================
 
/**
 * @brief 控制 nCS 引脚电平
 * @param state: GPIO_PIN_SET (高电平) or GPIO_PIN_RESET (低电平)
 */
static inline void ADS131_CS_Set(GPIO_PinState state)
{
    HAL_GPIO_WritePin(ADS131_CS_GPIO_Port, ADS131_CS_Pin, state);
}
 
/**
 * @brief 通过 SPI 发送和接收数据
 * @param pTxData: 发送数据缓冲区指针
 * @param pRxData: 接收数据缓冲区指针
 * @param size: 数据长度 (字节)
 * @return HAL_StatusTypeDef: HAL 状态
 */
static HAL_StatusTypeDef ADS131_SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t size)
{
    return HAL_SPI_TransmitReceive(ADS131_SPI_HANDLE, pTxData, pRxData, size, SPI_TIMEOUT);
}
 
 //==============================================================================
 // 主要功能函数实现
 //==============================================================================
 
/**
 * @brief 初始化 ADS131 驱动
 * @return bool: true 初始化成功, false 初始化失败
 */
bool ADS131_Init(void)
{
    // 确保 SPI 句柄不为空
    if (ADS131_SPI_HANDLE == NULL)
    {
        return false;
    }

    // 初始化时将 CS 引脚拉高
    ADS131_CS_Set(GPIO_PIN_SET);

    // 初始化寄存器本地副本为默认值
    ADS131_RestoreRegisterDefaults();
    
    return true;
}
 
/**
 * @brief 执行 ADS131 的启动序列
 * @return bool: true 启动成功, false 启动失败
 */
bool ADS131_Startup(void)
{
    // 等待电源稳定
    HAL_Delay(50);

    // 拉高 nRESET 使能 ADC
    ADS131_Reset();
    HAL_Delay(1);

    // 发送 NULL 建立 SPI 通信
    // 在上电复位(POR)后，第一个命令的响应是 0xFF04 (READY)，参考数据手册P79
    ADS131_SendCommand(OPCODE_NULL);
    HAL_Delay(1);

    // 再次发送 NULL 命令并验证响应
    uint16_t response = ADS131_SendCommand(OPCODE_NULL);
    if (response != 0xFF04)
    {
        return false;
    }

    // 发送 UNLOCK 解锁寄存器，参考数据手册P79
    if (ADS131_UnlockRegisters() == false)
    {
        return false; 
    }

    // 配置 D_SYS_CFG 寄存器
    uint8_t regVal = 0;
#ifdef SET_FIXED // 固定帧
    regVal |= D_SYS_CFG_FIXED_MASK;     // 设置 FIXED 位
#endif
#ifdef SET_CRC_EN // CRC
    regVal |= D_SYS_CFG_CRC_EN_MASK;    // 设置 CRC_EN 位
#endif
    ADS131_WriteSingleRegister(D_SYS_CFG_ADDRESS, regVal);//参考数据手册P62

   // 配置数据率和采样率，参考数据手册P63-P64
    // f_CLKIN = 16.384 MHz
    // f_DATA = (f_CLKIN / (CLK_DIV * ICLK_DIV)) / OSR = 16.384 MHz / (2 * 2) / 32 = 128 kHz
    ADS131_WriteSingleRegister(CLK1_ADDRESS, CLK1_CLK_DIV_2);
    ADS131_WriteSingleRegister(CLK2_ADDRESS, CLK2_ICLK_DIV_2 | CLK2_OSR_32);

    // 使能所有 ADC 通道, 参考数据手册P65
    ADS131_WriteSingleRegister(ADC_ENA_ADDRESS, ADC_ENA_ENA_ALL_CH_PWUP);

    // 发送 WAKEUP 命令开始转换, 参考数据手册P79
    ADS131_SendCommand(OPCODE_WAKEUP);
    
    return true;
}
 
/**
 * @brief 读取单个寄存器的值,参考数据手册P52
 * @param address: 要读取的寄存器地址
 * @return uint8_t: 读取到的寄存器值, 如果地址无效则返回 0
 */
uint8_t ADS131_ReadSingleRegister(uint8_t address)
{
    if (address >= NUM_REGISTERS)
    {
        return 0;
    }

    // 构建 RREG 命令
    spi_tx_data[0] = UPPER_BYTE(OPCODE_RREG) | address;
    spi_tx_data[1] = 0x00;
    for(uint8_t i = 2; i < WORD_LENGTH_BYTES; i++) spi_tx_data[i] = 0x00;

    // [帧 1] 发送 RREG 命令
    ADS131_CS_Set(GPIO_PIN_RESET);
    ADS131_SPI_TransmitReceive(spi_tx_data, spi_rx_data, WORD_LENGTH_BYTES);
    ADS131_CS_Set(GPIO_PIN_SET);

    HAL_Delay(1);

    // [帧 2] 发送 NULL 命令以获取寄存器数据
    uint16_t response = ADS131_SendCommand(OPCODE_NULL);

    // 验证响应并更新本地寄存器映射
    if (UPPER_BYTE(response) == (UPPER_BYTE(OPCODE_RREG) | address))
    {
        registerMap[address] = LOWER_BYTE(response);
    }

    return registerMap[address];
}
 
/**
 * @brief 向单个寄存器写入数据，参考数据手册P53
 * @param address: 要写入的寄存器地址
 * @param data: 要写入的 8 位数据
 */
void ADS131_WriteSingleRegister(uint8_t address, uint8_t data)
{
    if (address >= NUM_REGISTERS)
    {
        return; // 地址无效
    }

    // 如果寄存器已锁定，则无法写入
    if (registersLocked) return;

    uint8_t old_val = registerMap[address];

    // 构建 WREG 命令
    spi_tx_data[0] = UPPER_BYTE(OPCODE_WREG) | address;
    spi_tx_data[1] = data;
    for(uint8_t i = 2; i < WORD_LENGTH_BYTES; i++) spi_tx_data[i] = 0x00;

    // [帧 1] 发送 WREG 命令
    ADS131_CS_Set(GPIO_PIN_RESET);
    ADS131_SPI_TransmitReceive(spi_tx_data, spi_rx_data, WORD_LENGTH_BYTES);
    ADS131_CS_Set(GPIO_PIN_SET);

    // 预先更新本地寄存器值
    registerMap[address] = data;
    
    HAL_Delay(1);

    // [帧 2] 发送 NULL 命令以确认写入是否成功
    uint16_t response = ADS131_SendCommand(OPCODE_NULL);

    // 验证响应，如果响应不匹配，则恢复旧值
    if (UPPER_BYTE(response) == (UPPER_BYTE(OPCODE_RREG) | address) && LOWER_BYTE(response) == data)
    {
      // 写入成功
      registerMap[address] = LOWER_BYTE(response);
    }
    else
    {
        // 写入失败，恢复旧值
      registerMap[address] = old_val;
    }
}
 
/**
 * @brief 读取一次 ADC 转换数据 (所有通道)
 * @param dataStruct: 指向数据结构体的指针，用于存放结果
 * @return bool: 始终返回 true
 */
bool ADS131_ReadData(ads131_data_t *dataStruct)
{
    // 清空发送缓冲区 (发送 NULL)
    for (int i = 0; i < spi_frame_byte_len; ++i)
    {
        spi_tx_data[i] = 0x00;
    }

    // 执行 SPI 通信
    ADS131_CS_Set(GPIO_PIN_RESET);
    ADS131_SPI_TransmitReceive(spi_tx_data, spi_rx_data, spi_frame_byte_len);
    ADS131_CS_Set(GPIO_PIN_SET);

    // 解析接收到的数据
    dataStruct->response = COMBINE_BYTES(spi_rx_data[0], spi_rx_data[1]);

    // 逐个通道进行符号扩展
    // 数据帧偏移: 1个响应字
    dataStruct->channel1 = ADS131_SignExtend(&spi_rx_data[1 * WORD_LENGTH_BYTES]);
    dataStruct->channel2 = ADS131_SignExtend(&spi_rx_data[2 * WORD_LENGTH_BYTES]);
    dataStruct->channel3 = ADS131_SignExtend(&spi_rx_data[3 * WORD_LENGTH_BYTES]);
    dataStruct->channel4 = ADS131_SignExtend(&spi_rx_data[4 * WORD_LENGTH_BYTES]);

    // CRC 已禁用，不进行校验
    return true;
}
 
/**
 * @brief 发送一个 16 位的命令 (如 NULL, RESET, STANDBY, WAKEUP)
 * @param opcode: 16 位命令码
 * @return uint16_t: 设备的 16 位响应
 */
uint16_t ADS131_SendCommand(uint16_t opcode)
{
    // 构建命令
    spi_tx_data[0] = UPPER_BYTE(opcode);
    spi_tx_data[1] = LOWER_BYTE(opcode);
    for(uint8_t i = 2; i < WORD_LENGTH_BYTES; i++) spi_tx_data[i] = 0x00;

    // 发送命令
    ADS131_CS_Set(GPIO_PIN_RESET);
    ADS131_SPI_TransmitReceive(spi_tx_data, spi_rx_data, WORD_LENGTH_BYTES);
    ADS131_CS_Set(GPIO_PIN_SET);

    // 如果是复位命令，则恢复寄存器默认值
    if (opcode == OPCODE_RESET)
    {
        ADS131_RestoreRegisterDefaults();
        registersLocked = true;
    }

    // 返回响应
    return COMBINE_BYTES(spi_rx_data[0], spi_rx_data[1]);
}
 
/**
 * @brief 发送 LOCK 命令锁定寄存器
 * @return bool: true 如果锁定成功
 */
bool ADS131_LockRegisters(void)
{
    // [帧 1] 发送 LOCK 命令
    ADS131_SendCommand(OPCODE_LOCK);

    HAL_Delay(1);

    // [帧 2] 发送 NULL 命令以确认状态
    uint16_t response = ADS131_SendCommand(OPCODE_NULL);

    if (response == OPCODE_LOCK)
    {
        registersLocked = true;
    }
    else
    {
        registersLocked = false;
    }

    return registersLocked;
}
 
/**
 * @brief 发送 UNLOCK 命令解锁寄存器，参考数据手册P79
 * @return bool: true 如果解锁成功
 */
bool ADS131_UnlockRegisters(void)
{
    // [帧 1] 发送 UNLOCK 命令
    ADS131_SendCommand(OPCODE_UNLOCK);

    HAL_Delay(1);

    // [帧 2] 发送 NULL 命令以确认状态
    uint16_t response = ADS131_SendCommand(OPCODE_NULL);

    if (response == OPCODE_UNLOCK)
    {
        registersLocked = false;
    }
    else
    {
        registersLocked = true;
    }

    return !registersLocked;
}
 
/**
 * @brief 通过 nRESET 引脚硬件复位 ADC
 */
void ADS131_Reset(void)
{
    HAL_GPIO_WritePin(ADS131_RESET_GPIO_Port, ADS131_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(1); // 保持低电平一段时间
    HAL_GPIO_WritePin(ADS131_RESET_GPIO_Port, ADS131_RESET_Pin, GPIO_PIN_SET);
    
    // 复位后，寄存器恢复默认值
    ADS131_RestoreRegisterDefaults();
    registersLocked = true;
}
 
/**
 * @brief 获取内部存储的寄存器值
 * @param address: 寄存器地址
 * @return uint8_t: 寄存器值
 */
uint8_t ADS131_GetRegisterValue(uint8_t address)
{
    if (address >= NUM_REGISTERS)
    {
        return 0; // 地址无效
    }
    return registerMap[address];
}
 
/**
 * @brief 将内部寄存器映射数组恢复为默认值
 */
void ADS131_RestoreRegisterDefaults(void)
{
    registerMap[ID_MSB_ADDRESS]      = ID_MSB_NU_CH_4;
    registerMap[ID_LSB_ADDRESS]      = 0x00;
    registerMap[STAT_1_ADDRESS]      = STAT_1_DEFAULT;
    registerMap[STAT_P_ADDRESS]      = STAT_P_DEFAULT;
    registerMap[STAT_N_ADDRESS]      = STAT_N_DEFAULT;
    registerMap[STAT_S_ADDRESS]      = STAT_S_DEFAULT;
    registerMap[ERROR_CNT_ADDRESS]   = ERROR_CNT_DEFAULT;
    registerMap[STAT_M2_ADDRESS]     = STAT_M2_DEFAULT & STAT_M2_DEFAULT_MASK;
    registerMap[A_SYS_CFG_ADDRESS]   = A_SYS_CFG_DEFAULT;
    registerMap[D_SYS_CFG_ADDRESS]   = D_SYS_CFG_DEFAULT;
    registerMap[CLK1_ADDRESS]        = CLK1_DEFAULT;
    registerMap[CLK2_ADDRESS]        = CLK2_DEFAULT;
    registerMap[ADC_ENA_ADDRESS]     = ADC_ENA_DEFAULT;
    registerMap[ADC1_ADDRESS]        = ADC1_DEFAULT;
    registerMap[ADC2_ADDRESS]        = ADC2_DEFAULT;
    registerMap[ADC3_ADDRESS]        = ADC3_DEFAULT;
    registerMap[ADC4_ADDRESS]        = ADC4_DEFAULT;
}
 
/**
 * @brief 将接收到的字节数组进行符号扩展，转换为32位有符号整数
 * @param dataBytes: 指向数据字节数组的指针 (MSB 在前)
 * @return int32_t: 转换后的 32 位有符号整数
 */
int32_t ADS131_SignExtend(const uint8_t dataBytes[])
{
#ifdef WORD_LENGTH_24BIT
    // 对于 24 位数据
    int32_t val = ((int32_t)dataBytes[0] << 24) |
                  ((int32_t)dataBytes[1] << 16) |
                  ((int32_t)dataBytes[2] << 8);

    // 通过算术右移来扩展符号位
    return (val >> 8);

#elif defined WORD_LENGTH_16BIT_TRUNCATED
    // 对于 16 位数据
    int32_t val = ((int32_t)dataBytes[0] << 24) |
                  ((int32_t)dataBytes[1] << 16);
    return (val >> 16);

#else
    // 其他位宽暂未实现
    return 0;
#endif
}
