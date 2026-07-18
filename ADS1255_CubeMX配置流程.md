已根据 `TempCubeMX1` 工程整理 ADS1255/ADS1256 配置流程。本工程采用 **GPIO 软件模拟串行通信** 读取 ADC，**没有使用 STM32 硬件 SPI、SPI DMA 或 SPI 中断**。

工程中的 ADS 驱动文件为：

```text
USER/ads1255.h
USER/ads1255.c
```

虽然文件名为 `ads1255`，当前函数名称和通道宏统一使用：

```text
ADS1256_xxx
AIN0 ～ AIN7
```

因此使用前应先确认电路板上安装的具体芯片型号，以及模拟输入通道数量是否与驱动一致。

---

## CubeMX 配置核心流程

### 1. ADS 与 STM32 引脚对应关系

当前工程使用五个 GPIO：

```text
PC4  -> ADS DRDY
PA4  -> ADS CS
PA6  -> ADS DOUT
PA7  -> ADS DIN
PA5 -> ADS SCLK
```

对应的 CubeMX User Label 为：

```text
PC4 : ADCDRDY
PA4 : ADCCS
PA6 : ADCDOUT
PA7 : ADCDIN
PA5 : ADCSCLK
```

硬件连接建议：

```text
STM32 PC4  -> ADS DRDY
STM32 PA4  -> ADS CS
STM32 PA6  -> ADS DOUT
STM32 PA7  -> ADS DIN
STM32 PA5  -> ADS SCLK
STM32 GND  -> ADS DGND/AGND
```

> STM32 与 ADS 必须共地。  
> ADS 的参考电压、模拟电源、数字电源、外部时钟、RESET、SYNC/PDWN 等引脚，应按原理图连接到确定状态，不能悬空。  

---

### 2. PC4 配置为 ADCDRDY

在 System Core -> GPIO 页面选择`PC4`：

```text
GPIO_Input
User Label: ADCDRDY
Pull: No pull
```

当前工程生成代码：

```c
GPIO_InitStruct.Pin = ADCDRDY_Pin | ADCDOUT_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
```

DRDY 为低电平时表示新转换数据已经准备好。

驱动中的判断宏为：

```c
#define ADS_DRDY_IS_LOW() \
    (HAL_GPIO_ReadPin(ADS_DRDY_PORT, ADS_DRDY_PIN) == GPIO_PIN_RESET)
```

如果硬件上没有可靠上拉或信号存在悬空现象，可根据电路实际情况设置外部上拉。通常应优先按照芯片手册和原理图确定，而不是盲目开启 MCU 内部上拉。

---

### 3. PA5 配置为 ADCCS

在 System Core -> GPIO页面选择 `PA5`：

```text
GPIO_Output
User Label: ADCCS
GPIO output level: High
GPIO mode: Output Push Pull
Pull: No pull
Maximum output speed: Low
```

当前工程生成代码：

```c
HAL_GPIO_WritePin(ADCCS_GPIO_Port,
                  ADCCS_Pin,
                  GPIO_PIN_SET);

GPIO_InitStruct.Pin = ADCCS_Pin | ADCDIN_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
```



---

### 4. PA6 配置为 ADCDOUT

在 System Core -> GPIO页面选择 `PA6`：

```text
GPIO_Input
User Label: ADCDOUT
Pull: No pull
```

---

### 5. PA7 配置为 ADCDIN

在 System Core -> GPIO页面选择 `PA7`：

```text
GPIO_Output
User Label: ADCDIN
GPIO output level: Low
GPIO mode: Output Push Pull
Pull: No pull
Maximum output speed: Low
```

---

### 6. PA4 配置为 ADCSCLK

在 CubeMX Pinout 页面选择 `PA4`：

```text
GPIO_Output
User Label: ADCSCLK
GPIO output level: Low
GPIO mode: Output Push Pull
Pull: No pull
Maximum output speed: Low
```

当前工程生成代码：

```c
HAL_GPIO_WritePin(ADCSCLK_GPIO_Port,
                  ADCSCLK_Pin,
                  GPIO_PIN_RESET);

GPIO_InitStruct.Pin = ADCSCLK_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(ADCSCLK_GPIO_Port, &GPIO_InitStruct);
```

---

## 驱动文件配置

### 放置文件

### ads1255.h

```c
#ifndef __ADS1256_H
#define __ADS1256_H

#include "main.h"
#include "gpio.h"

/* ????? */
#define REG_STATUS	(0)
#define	REG_MUX    	(1)
#define	REG_ADCON   (2)
#define	REG_DRATE   (3)
#define	REG_IO      (4)

/* ???? */
#define	CMD_WAKEUP		(0x00)
#define	CMD_RDATA   	(0x01)
#define	CMD_RDATAC  	(0x03)
#define	CMD_SDATAC  	(0x0F)
#define	CMD_RREG    	(0x10)
#define	CMD_WREG    	(0x50)
#define	CMD_SELFCAL 	(0xF0)
#define	CMD_SELFOCAL	(0xF1)
#define	CMD_SELFGCAL	(0xF2)
#define	CMD_SYSOCAL 	(0xF3)
#define	CMD_SYSGCAL 	(0xF4)
#define	CMD_SYNC    	(0xFC)
#define	CMD_STANDBY 	(0xFD)
#define	CMD_RESET   	(0xFE)

/* PGA?? */
#define PGA_1            0x00
#define PGA_2            0x01
#define PGA_4            0x02
#define PGA_8            0x03
#define PGA_16           0x04
#define PGA_32           0x05
#define PGA_64           0x06

/* ??? */
#define DATARATE_30K         0xF0
#define DATARATE_15K         0xE0
#define DATARATE_7_5K        0xD0
#define DATARATE_3_7_5K      0xC0
#define DATARATE_2K          0xB0
#define DATARATE_1K          0xA0
#define DATARATE_500         0x92
#define DATARATE_100         0x82
#define DATARATE_60        	 0x72
#define DATARATE_50        	 0x63
#define DATARATE_30        	 0x53
#define DATARATE_25        	 0x43
#define DATARATE_15        	 0x33
#define DATARATE_10        	 0x23
#define DATARATE_5        	 0x13
#define DATARATE_2_5     	 0x02

/* ????? */
#define POSITIVE_AIN0       (0X00)
#define POSITIVE_AIN1       (0X10)
#define POSITIVE_AIN2       (0X20)
#define POSITIVE_AIN3       (0X30)
#define POSITIVE_AIN4       (0X40)
#define POSITIVE_AIN5       (0X50)
#define POSITIVE_AIN6       (0X60)
#define POSITIVE_AIN7       (0X70)
#define POSITIVE_AINCOM     (0X80)

#define NEGTIVE_AIN0         0X00
#define NEGTIVE_AIN1         0X01
#define NEGTIVE_AIN2         0X02
#define NEGTIVE_AIN3         0X03
#define NEGTIVE_AIN4         0X04
#define NEGTIVE_AIN5         0X05
#define NEGTIVE_AIN6         0X06
#define NEGTIVE_AIN7         0X07
#define NEGTIVE_AINCOM       0X08

/* ADS1255/ADS1256 Òý½Å¶¨Òå  */
#define ADS_SCLK_PORT   ADCSCLK_GPIO_Port
#define ADS_SCLK_PIN    ADCSCLK_Pin

#define ADS_DIN_PORT    ADCDIN_GPIO_Port
#define ADS_DIN_PIN     ADCDIN_Pin

#define ADS_DOUT_PORT   ADCDOUT_GPIO_Port
#define ADS_DOUT_PIN    ADCDOUT_Pin

#define ADS_DRDY_PORT   ADCDRDY_GPIO_Port
#define ADS_DRDY_PIN    ADCDRDY_Pin

#define ADS_CS_PORT     ADCCS_GPIO_Port
#define ADS_CS_PIN      ADCCS_Pin

/* GPIO??? */
#define ADS_SCLK_H      HAL_GPIO_WritePin(ADS_SCLK_PORT, ADS_SCLK_PIN, GPIO_PIN_SET)
#define ADS_SCLK_L      HAL_GPIO_WritePin(ADS_SCLK_PORT, ADS_SCLK_PIN, GPIO_PIN_RESET)

#define ADS_DIN_H       HAL_GPIO_WritePin(ADS_DIN_PORT, ADS_DIN_PIN, GPIO_PIN_SET)
#define ADS_DIN_L       HAL_GPIO_WritePin(ADS_DIN_PORT, ADS_DIN_PIN, GPIO_PIN_RESET)

#define ADS_CS_H        HAL_GPIO_WritePin(ADS_CS_PORT, ADS_CS_PIN, GPIO_PIN_SET)
#define ADS_CS_L        HAL_GPIO_WritePin(ADS_CS_PORT, ADS_CS_PIN, GPIO_PIN_RESET)

#define ADS_DRDY_IS_LOW() \
    (HAL_GPIO_ReadPin(ADS_DRDY_PORT, ADS_DRDY_PIN) == GPIO_PIN_RESET)

/* º¯Êý */
void ADS1256_GPIO_Init(void);
void ADS1256_CfgADC(uint8_t gain, uint8_t drate);

void ADS1256_SetSingleChannel(uint8_t positive);
void ADS1256_SetDiffChannel(uint8_t positive, uint8_t negative);

//uint32_t ADS1256_GetAdc(uint8_t channel);
int32_t ADS1256_GetAdcSigned(void);

uint8_t ADS1256_ReadReg(uint8_t reg);
void ADS1256_WriteReg(uint8_t reg, uint8_t value);

double ADS1256_CodeToVoltage(int32_t code, double vref, uint8_t gain);


#endif
```



### ads1255.c

```c
#include "ads1255.h"


static void delay_us(uint32_t us)
{
    volatile uint32_t count = us * (168 / 7);
    while(count--);
}
//GPIO???
void ADS1256_GPIO_Init(void)
{
    /* GPIO ??? MX_GPIO_Init() ?? CubeMX ??? */

    ADS_CS_H;
    ADS_SCLK_L;
    ADS_DIN_H;
}


//??????
static void ADS1256_Send8Bit(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
            ADS_DIN_H;
        else
            ADS_DIN_L;

        ADS_SCLK_H;
        delay_us(1);
        data <<= 1;
        ADS_SCLK_L; /* ADS1256?SCLK?????DIN */
        delay_us(1);
    }
}

static uint8_t ADS1256_Recive8Bit(void)
{
    uint8_t read = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        read <<= 1;
        ADS_SCLK_H;
        delay_us(1);
        if (HAL_GPIO_ReadPin(ADS_DOUT_PORT, ADS_DOUT_PIN) == GPIO_PIN_SET)
            read |= 0x01;
        ADS_SCLK_L;
        delay_us(1);
    }
    return read;
}

void ADS1256_WriteCmd(uint8_t cmd)
{
    ADS_CS_L;
    ADS1256_Send8Bit(cmd);
    ADS_CS_H;
}

void ADS1256_WriteReg(uint8_t reg, uint8_t value)
{
    ADS_CS_L;
    ADS1256_Send8Bit(CMD_WREG | reg);
    ADS1256_Send8Bit(0x00);  /* ?1???? */
    ADS1256_Send8Bit(value);
    ADS_CS_H;
}

void ADS1256_SetDiffChannel(uint8_t positive, uint8_t negative)
{
    uint8_t mux;
    uint32_t timeout = 5000000;

    mux = (positive & 0xF0) | (negative & 0x0F);

    ADS1256_WriteReg(REG_MUX, mux);

    delay_us(10);

    ADS1256_WriteCmd(CMD_SYNC);
    delay_us(10);

    ADS1256_WriteCmd(CMD_WAKEUP);
    delay_us(10);

    while (!ADS_DRDY_IS_LOW())
    {
        if (--timeout == 0)
        {
            return;
        }
    }
}


void ADS1256_SetSingleChannel(uint8_t positive)
{
    ADS1256_SetDiffChannel(positive, NEGTIVE_AINCOM);
}



uint8_t ADS1256_ReadReg(uint8_t reg)
{
    uint8_t read;

    ADS_CS_L;
    ADS1256_Send8Bit(CMD_RREG | reg);
    ADS1256_Send8Bit(0x00);
    delay_us(5);
    read = ADS1256_Recive8Bit();
    ADS_CS_H;

    return read;
}

void ADS1256_CfgADC(uint8_t gain, uint8_t drate)
{
    uint8_t buf[4];

    ADS1256_WriteCmd(CMD_RESET);
    HAL_Delay(5);

    ADS1256_WriteCmd(CMD_SDATAC);
    delay_us(10);

    buf[0] = 0x00;                                    // STATUS: ORDER=MSB, ACAL=0, BUFEN=0
    buf[1] = POSITIVE_AIN0 | NEGTIVE_AIN1;            // MUX: AIN0 - AIN1
    buf[2] = gain & 0x07;                             // ADCON: CLKOUT off, SDCS off, PGA
    buf[3] = drate;                                   // DRATE: ?? 0x82 = 100SPS

    ADS_CS_L;
    ADS1256_Send8Bit(CMD_WREG | 0x00);
    ADS1256_Send8Bit(0x03);     // ?4????:STATUS/MUX/ADCON/DRATE
    ADS1256_Send8Bit(buf[0]);
    ADS1256_Send8Bit(buf[1]);
    ADS1256_Send8Bit(buf[2]);
    ADS1256_Send8Bit(buf[3]);
    ADS_CS_H;

    delay_us(10);

    ADS1256_WriteCmd(CMD_SELFCAL);

    while (!ADS_DRDY_IS_LOW())
    {
        // ?????
    }
}

int32_t ADS1256_GetAdcSigned(void)
{
    uint32_t read = 0;
    uint32_t timeout = 5000000;

    while (!ADS_DRDY_IS_LOW())
    {
        if (--timeout == 0)
        {
            return 0x7FFFFFFF;   // ?????
        }
    }

    ADS_CS_L;

    ADS1256_Send8Bit(CMD_RDATA);

    delay_us(10);

    read  = (uint32_t)ADS1256_Recive8Bit() << 16;
    read |= (uint32_t)ADS1256_Recive8Bit() << 8;
    read |= (uint32_t)ADS1256_Recive8Bit();

    ADS_CS_H;

    // 24 Î»²¹Âë·ûºÅÀ©Õ¹µ½ 32Î» ?
    if (read & 0x800000)
    {
        read |= 0xFF000000;
    }

    return (int32_t)read;
}


double ADS1256_CodeToVoltage(int32_t code, double vref, uint8_t gain)
{
    uint8_t pga;

    switch (gain)
    {
        case PGA_1:  pga = 1;  break;
        case PGA_2:  pga = 2;  break;
        case PGA_4:  pga = 4;  break;
        case PGA_8:  pga = 8;  break;
        case PGA_16: pga = 16; break;
        case PGA_32: pga = 32; break;
        case PGA_64: pga = 64; break;
        default:     pga = 1;  break;
    }

    return ((double)code * 2.0 * vref) / ((double)pga * 8388607.0);
}


/*uint32_t ADS1256_GetAdc(uint8_t channel)
{
    uint32_t read = 0;
    uint32_t timeout = 5000000;

    (void)channel;  // ????? AIN0-AINCOM,channel ????

    // ?? DRDY ??,????????
    while (!ADS_DRDY_IS_LOW())
    {
        if (--timeout == 0)
        {
            return 0xFFFFFFFF;
        }
    }

    ADS_CS_L;

    ADS1256_Send8Bit(CMD_RDATA);

    // ?????? RDATA ???? t6,7.68MHz ?? 6.5us,? 10us ??
    delay_us(10);

    read  = (uint32_t)ADS1256_Recive8Bit() << 16;
    read |= (uint32_t)ADS1256_Recive8Bit() << 8;
    read |= (uint32_t)ADS1256_Recive8Bit();

    ADS_CS_H;

    return read;
} */


uint8_t ADS1256_GetChipID(void)
{
    uint8_t id = ADS1256_ReadReg(REG_STATUS);
    return (id >> 4);
}
```

### 文件放置位置

当前工程把 ADS 驱动放在：

```text
USER/ads1255.h
USER/ads1255.c
```

如果仍放在 `USER` 目录，需要确认：

```text
USER 目录已经加入编译器 Include Path
ads1255.c 已加入工程并参与编译
```

---

## main 中调用例子

不要整文件覆盖 CubeMX 生成的 `main.c`，只把代码放到对应的 `USER CODE` 区域。

### Includes 区域

```c
/* USER CODE BEGIN Includes */
#include "ads1255.h"
/* USER CODE END Includes */
```

---

### 全局变量区域

```c
/* USER CODE BEGIN PV */

volatile int32_t ads_raw = 0;
volatile uint32_t ads_raw24 = 0;
volatile float ads_voltage = 0.0f;


#define ADS_VREF   2.5f
#define ADS_PGA    1.0f

char ads_raw_binary[33];

/* USER CODE END PV */
```

参数说明：

```text
ads_raw24   : ADS 返回的原始 24 位数据
ads_raw     : 符号扩展后的 int32_t 数据
ads_voltage : 换算后的差分电压
ADS_VREF    : 实际参考电压
ADS_PGA     : 当前 PGA 增益
```

> `ADS_VREF` 必须填写硬件上真实的参考电压。  
> 如果参考标称为 2.5 V，但实际测量并非精确 2.500000 V，软件换算会产生比例误差。

---

### 初始化区域

必须先调用 CubeMX 生成的 GPIO 初始化：

```c
MX_GPIO_Init();
```

然后在 `USER CODE BEGIN 2` 中：

```c
/* USER CODE BEGIN 2 */

ADS1256_GPIO_Init();
ADS1256_CfgADC(PGA_1, DATARATE_50);

/* USER CODE END 2 */
```

完整顺序示例：

```c
MX_GPIO_Init();

/* USER CODE BEGIN 2 */

ADS1256_GPIO_Init();
ADS1256_CfgADC(PGA_1, DATARATE_50);

ADS1256_SetDiffChannel(POSITIVE_AIN0,
                       NEGTIVE_AIN1);

/* USER CODE END 2 */
```

> `ADS1256_GPIO_Init()` 必须放在 `MX_GPIO_Init()` 之后。  
> 驱动中的 GPIO 初始化函数只设置初始电平，不负责开启 GPIO 时钟，也不负责配置输入输出模式。

---

### while循环中

在进入 `while (1)` 前定义局部变量：

```c
/* USER CODE BEGIN WHILE */

int32_t code;
uint32_t raw24;
double voltage;

while (1)
{
```

循环内部只保留 ADS 采集：

```c
    code = ADS1256_GetAdcSigned();
		raw24 = ((uint32_t)code) & 0x00FFFFFF;

    if (code != 0x7FFFFFFF)
    {
        voltage = ADS1256_CodeToVoltage(code, 2.5, PGA_1);

        ads_raw = code;
        ads_voltage = (float)voltage;
			/*
			* ÏÔÊ¾Ô­Ê¼ 24bit ADC code?
         * signed code ?? 24 ??? ADS1256 ??????
         */
        ads_raw24 = ((uint32_t)code) & 0x00FFFFFF;
        Uint32_ToBinaryString(ads_raw24, ads_raw_binary);
    }
```

---

## 输出二进制 ADC Code

当前工程使用：

```c
/* USER CODE BEGIN 0 */
static void Uint32_ToBinaryString(uint32_t value,
                                  char output[33])
{
    for (uint32_t i = 0U; i < 32U; i++)
    {
        uint32_t bit_pos = 31U - i;

        output[i] =
            ((value >> bit_pos) & 0x01U) ?
            '1' : '0';
    }

    output[32] = '\0';
}
/* USER CODE END 0 */
```

调用：

```c
Uint32_ToBinaryString((uint32_t)ads_raw,
                      ads_raw_binary);
```

这会输出 32 位字符串：

```text
ADC_Code=00000000000110011001101001101001
```

