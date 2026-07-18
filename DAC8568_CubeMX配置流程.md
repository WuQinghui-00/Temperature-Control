

本工程采用 **GPIO 模拟串行通信** 驱动 DAC8568，**没有使用 STM32 硬件 SPI、DMA 或 SPI 中断**。

工程中的 DAC 相关文件为：

```text
USER/bsp_DAC8568.h
USER/bsp_DAC8568.c
```

## CubeMX 配置核心流程

### 1. DAC8568 与 STM32 引脚对应关系

当前工程使用以下三个 GPIO：

```text
PE10 -> DAC8568 SCLK
PE11 -> DAC8568 DIN
PE12 -> DAC8568 SYNC
```

对应的 CubeMX User Label 为：

```text
PE10: DACSCLK
PE11: DACDIN
PE12: DACSYNC
```

硬件连接建议：

```text
STM32 PE10  -> DAC8568 SCLK
STM32 PE11  -> DAC8568 DIN
STM32 PE12  -> DAC8568 SYNC
STM32 GND   -> DAC8568 GND
```

> DAC8568 的数字地和 STM32 地必须共地。  
> `LDAC-GND`、`CLR-3.3V`、
>
> 基准与电源引脚没有在当前工程中由 STM32 GPIO 控制，应根据原理图连接到确定电平，不能悬空。

---

### 2. PE10 配置为 DACSCLK

在System Core -> GPIO 页面选择 `PE10`：

配置如下：

```text
GPIO_Output
User Label: DACSCLK
GPIO output level: Low
GPIO mode: Output Push Pull
Pull: No pull
Maximum output speed: Very High
```

工程生成代码对应：

```c
HAL_GPIO_WritePin(GPIOE, DACSCLK_Pin, GPIO_PIN_RESET);

GPIO_InitStruct.Pin = DACSCLK_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
```



---

### 3. PE11 配置为 DACDIN

在 System Core -> GPIO 页面选择 `PE11`：

```text
GPIO_Output
User Label: DACDIN
GPIO output level: Low
GPIO mode: Output Push Pull
Pull: No pull
Maximum output speed: Very High
```

工程生成代码对应：

```c
HAL_GPIO_WritePin(GPIOE, DACDIN_Pin, GPIO_PIN_RESET);

GPIO_InitStruct.Pin = DACDIN_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
```

---

### 4. PE12 配置为 DACSYNC

在 System Core -> GPIO页面选择 `PE12`：

```text
GPIO_Output
User Label: DACSYNC
GPIO output level: High
GPIO mode: Output Push Pull
Pull: No pull
Maximum output speed: Very High
```

LDAC 和 CLR 不需要在 CubeMX 里配置。

工程生成代码对应：

```c
HAL_GPIO_WritePin(DACSYNC_GPIO_Port,
                  DACSYNC_Pin,
                  GPIO_PIN_SET);

GPIO_InitStruct.Pin = DACSYNC_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
HAL_GPIO_Init(DACSYNC_GPIO_Port, &GPIO_InitStruct);
```



---

## 驱动文件配置

### bsp_DAC8568.h

```c
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

#define DAC8568_CODE_MAX        65535U
#define DAC8568_FULL_SCALE_MV   5000U


void DAC8568_Init(void);
void SPI_SendHalfWord(uint16_t m);
void SPI_SendByte(uint8_t m);
void DAC8568_SetVoltage(unsigned char mCh, float mVol);
void DAC8568_SetDAC(unsigned char mCh, int DAC_Set);

extern volatile uint16_t g_dac8568_code[8];




#endif
```



### bsp_DAC8568.c

```c
#include "bsp_DAC8568.h"


///*******************************************
//		函数名称：iDAC8568_GPIO_Init
//		功    能：初始化DAC8568IO
//		入口参数：无
//		返回值  ：无
//********************************************/
static void DAC8568_GPIO_Init(void)
{

	DAC8568_SYNC_H;
	DAC8568_SCLK_H;
	DAC8568_DIN_L;
	HAL_Delay(10);
}

void Mcpdely(void)
{
	  unsigned int i=10;while(i--);
}

//-----------------------------------------------------------------//
//	功    能：  模拟16位SPI通信，发送控制命令
//	入口参数: /	发送的SPI数据
//	出口参数: /	接收的SPI数据
//	全局变量: /
//	备    注: 	发送函数
//-----------------------------------------------------------------//
void SPI_SendHalfWord(uint16_t m)
{
    uint8_t i;

    for(i=0;i<16;i++)
    {
      DAC8568_SCLK_H;			 //clk上升沿读取dout数据
      if(m & 0x8000)
	  {
		DAC8568_DIN_H;
	  }
	  else
	  {
		DAC8568_DIN_L;
	  }
	  m = m<<1;
	  DAC8568_SCLK_L;			//clk下降沿把din上的数据传到ad
	  Mcpdely();
	}
}

//-----------------------------------------------------------------//
//	功    能：  模拟8位SPI通信，发送控制命令
//	入口参数: /	发送的SPI数据
//	出口参数: /	接收的SPI数据
//	全局变量: /
//	备    注: 	发送函数
//-----------------------------------------------------------------//
void SPI_SendByte(uint8_t m)
{
    uint8_t i;

    for(i=0;i<8;i++)
    {
      DAC8568_SCLK_H;			 //clk上升沿读取dout数据
      if(m & 0x80)
	  {
		DAC8568_DIN_H;
	  }
	  else
	  {
		DAC8568_DIN_L;
	  }
	  m = m<<1;
	  DAC8568_SCLK_L;			//clk下降沿把din上的数据传到ad
	  Mcpdely();
	}
}
/*
*********************************************************************************************************
*	函 数 名: DAC8562_WriteCmd
*	功能说明: 通过SPI总线发送24bit数据。
*	参    数: _cmd : 命令
*	返 回 值: 无
*********************************************************************************************************
*/
void DAC8568_WriteCmd(uint32_t _cmd)
{
	uint8_t i;

	DAC8568_SYNC_L;

	/*由于DAC8562 SCLK时钟高达50M，因此可以不需要延迟 */
	for(i = 0; i < 24; i++)
	{
		if (_cmd & 0x800000)
		{
			DAC8568_DIN_H;
		}
		else
		{
			DAC8568_DIN_L;
		}
		DAC8568_SCLK_H;
		_cmd <<= 1;
		DAC8568_SCLK_L;
	}

	DAC8568_SYNC_H;
}


 /**
  * @brief  DAC8568寄存器写入32bit数据
  * @param  PreConbyte: 4bit-Prefix bits + 4bit-control bits
  * @param  Addressbyte：4bit-Address bits
  * @param  Datashort: 16bit-Data bits
  * @param  Featurebyte：4bit-Feature bits
  */
static uint32_t ChatToInt(uint8_t PreConbyte, uint8_t Addressbyte, uint16_t Datashort, uint8_t Featurebits)
{
	uint32_t ret_val = 0;

	Addressbyte &= 0x0f;
	Featurebits &= 0x0f;
	ret_val = PreConbyte;
	ret_val <<= 4;
	ret_val |= Addressbyte;
	ret_val <<= 16;
	ret_val |= Datashort;
	ret_val <<= 4;
	ret_val |= Featurebits;

	return ret_val;
}



 /**
  * @brief  DAC8568指定通道写数据
  * @param  Addressbyte: 0-7，对应通道A到通道H
  * @param  Datashort：寄存器SR的32 bit数据
  */
static void DAC8568_Write_passageway(uint8_t Addressbyte, uint16_t Datashort)
{
	uint32_t SRData;    //发送给DA8568移位寄存器SR的值
	SRData = ChatToInt(PrefixControlbyte, Addressbyte, Datashort, Featurebyte);
	DAC8568_SYNC_L;
	Mcpdely();
	SPI_SendByte((SRData & 0xFF000000) >> 24);       //发送DB31-DB24位
	SPI_SendByte((SRData & 0xFF0000) >> 16);        //发送DB23-DB16位
	SPI_SendByte((SRData & 0xFF00) >> 8);          //发送DB15-DB8位
	SPI_SendByte(SRData & 0xFF);                   //发送DB7-DB0位
	DAC8568_SYNC_H;
	Mcpdely();
}





 /**
  * @brief  DAC8568写指定命令
  * @param  Addressbyte: 0-7，对应通道A到通道H
  * @param  Datashort：寄存器SR的32 bit数据
  */
static void DAC8568_Write_Command(uint8_t command)
{
	switch(command)
	{
		//内部基准电压上电 - 静态模式
		//注意：当 DAC 上电时，基准电压掉电；任何DAC上电时，基准电压才上电
		case SETUP_INTERNAL_REGISTER:
		{
			DAC8568_SYNC_L;
			SPI_SendByte(0x08);       //发送DB31-DB24位
			SPI_SendByte(0);          //发送DB23-DB16位
			SPI_SendByte(0);          //发送DB15-DB8位
			SPI_SendByte(0x01);       //发送DB7-DB0位
			DAC8568_SYNC_H;
			break;
		}

		//通道将相应位置设为1，分别为DAC A、B、C、D、E、F、G、H通道
		case POWER_UP:
		{
			DAC8568_SYNC_L;
			SPI_SendByte(0x04);       //发送DB31-DB24位
			SPI_SendByte(0);          //发送DB23-DB16位
			SPI_SendByte(0);          //发送DB15-DB8位
			SPI_SendByte(0xff);       //发送DB7-DB0位
			DAC8568_SYNC_H;
			break;
		}
		//复位
		case DAC8568RESET:
		{
			DAC8568_SYNC_L;
			SPI_SendByte(0x07);       //发送DB31-DB24位
			SPI_SendByte(0);          //发送DB23-DB16位
			SPI_SendByte(0);          //发送DB15-DB8位
			SPI_SendByte(0);       //发送DB7-DB0位
			DAC8568_SYNC_H;
			break;
		}
	}
}

/**
  * @brief  设置DAC8568通道 电压
  * @param  mCH:通道号0-7
  * @param  mVol：设置的电压值
  */
void  DAC8568_SetVoltage(unsigned char mCh,float mVol)
{
	float mDatafloat;
	uint16_t mDtashort;


	mDatafloat = mVol * (65535.0f / 5.0f);  // 2.5V * 2x增益 = 5V满量程  
	mDtashort = (uint16_t)mDatafloat;
	mDtashort &= 0xffff;
	if(mDtashort > 65535)
		mDtashort = 65535;

	DAC8568_SYNC_H;
	Mcpdely();
	switch(mCh)
	{
		case 0:      //DA8568的A通道
		{
			DAC8568_Write_passageway(AddressOutA, mDtashort);
			break;
		}
		case 1:      //DA8568的B通道
		{
			DAC8568_Write_passageway(AddressOutB, mDtashort);
			break;
		}
		case 2:      //DA8568的C通道
		{
			DAC8568_Write_passageway(AddressOutC, mDtashort);
			break;
		}
		case 3:      //DA8568的D通道
		{
			DAC8568_Write_passageway(AddressOutD, mDtashort);
			break;
		}
		case 4:      //DA8568的E通道
		{
			DAC8568_Write_passageway(AddressOutE, mDtashort);
			break;
		}
		case 5:      //DA8568的F通道
		{
			DAC8568_Write_passageway(AddressOutF, mDtashort);
			break;
		}
		case 6:      //DA8568的G通道
		{
			DAC8568_Write_passageway(AddressOutG, mDtashort);
			break;
		}
		case 7:      //DA8568的H通道
		{
			DAC8568_Write_passageway(AddressOutH, mDtashort);
			break;
		}
	}
	g_dac8568_code[mCh] = mDtashort;
}
/**
  * @brief  设置DAC8568通道 电压
  * @param  mCH:通道号0-7
  * @param  DAC_Set：设置的DAC值
  */
void  DAC8568_SetDAC(unsigned char mCh,int DAC_Set)
{

	uint16_t dac_code;
	
	
	if (mCh > 7)
    {
        return;
    }

    if (DAC_Set < 0)
    {
        dac_code = 0;
    }
    else if (DAC_Set > 65535)
    {
        dac_code = 65535;
    }
    else
    {
        dac_code = (uint16_t)DAC_Set;
    }
	
	DAC8568_SYNC_H;
	Mcpdely();
	switch(mCh)
	{
		case 0:      //DA8568的A通道
		{
			DAC8568_Write_passageway(AddressOutA, DAC_Set);
			break;
		}
		case 1:      //DA8568的B通道
		{
			DAC8568_Write_passageway(AddressOutB, DAC_Set);
			break;
		}
		case 2:      //DA8568的C通道
		{
			DAC8568_Write_passageway(AddressOutC, DAC_Set);
			break;
		}
		case 3:      //DA8568的D通道
		{
			DAC8568_Write_passageway(AddressOutD, DAC_Set);
			break;
		}
		case 4:      //DA8568的E通道
		{
			DAC8568_Write_passageway(AddressOutE, DAC_Set);
			break;
		}
		case 5:      //DA8568的F通道
		{
			DAC8568_Write_passageway(AddressOutF, DAC_Set);
			break;
		}
		case 6:      //DA8568的G通道
		{
			DAC8568_Write_passageway(AddressOutG, DAC_Set);
			break;
		}
		case 7:      //DA8568的H通道
		{
			DAC8568_Write_passageway(AddressOutH, DAC_Set);
			break;
		}

	}  
	g_dac8568_code[mCh] = dac_code;

}

//DA8568初始化
void DAC8568_Init(void)
{
	DAC8568_GPIO_Init();
	DAC8568_Write_Command(DAC8568RESET);                      //复位
	DAC8568_Write_Command(POWER_UP);                   //通道将相应位置设为1，分别为DAC A、B、C、D、E、F、G、H通道
	DAC8568_Write_Command(SETUP_INTERNAL_REGISTER);    //内部基准电压上电 - 静态模式

	/* Power up DAC-A and DAC-B */
	DAC8568_WriteCmd((4 << 19) | (0 << 16) | (3 << 0));

	/* LDAC pin inactive for DAC-B and DAC-A  不使用LDAC引脚控制更新 */
	DAC8568_WriteCmd((6 << 19) | (0 << 16) | (3 << 0));

	/* 选择内部基准并复位2个DAC的增益=2 （复位时内部基准是禁止的) */
	DAC8568_WriteCmd((7 << 19) | (0 << 16) | (1 << 0));
}
```





### 文件放置位置

当前工程把驱动放在：

```text
USER/bsp_DAC8568.h
USER/bsp_DAC8568.c
```

如果放在 `USER` 目录，需要确认：

```text
USER 目录已加入编译器 Include Path
bsp_DAC8568.c 已加入工程并参与编译
```

---

## main 中调用例子

不要修改文件覆盖 CubeMX 生成的 `main.c`，只把代码放入对应的 `USER CODE` 区域。

### Includes 区域

```c
/* USER CODE BEGIN Includes */
#include "bsp_DAC8568.h"
/* USER CODE END Includes */
```

---

### 全局变量区域

驱动会记录各通道最近一次写入的 DAC 码值：

```c
/* USER CODE BEGIN PV */
volatile uint16_t g_dac8568_code[8] = {0};
/* USER CODE END PV */
```

`bsp_DAC8568.h` 中已有外部声明：

```c
extern volatile uint16_t g_dac8568_code[8];
```

全局变量只能在一个 `.c` 文件中定义一次，不能在多个文件中重复定义。

---

### 初始化区域

必须先执行 CubeMX 生成的 GPIO 初始化：

```c
MX_GPIO_Init();
```

然后在 `USER CODE BEGIN 2` 中初始化 DAC：

```c
/* USER CODE BEGIN 2 */

DAC8568_Init();

/* A 通道输出 1.500 V */
DAC8568_SetVoltage(OutA, 1.5f);

/* C 通道输出 1.000 V */
DAC8568_SetVoltage(OutC, 1.0f);

/* USER CODE END 2 */
```

完整顺序示例：

```c
MX_GPIO_Init();

/* USER CODE BEGIN 2 */

DAC8568_Init();
DAC8568_SetVoltage(OutA, 1.5f);
DAC8568_SetVoltage(OutC, 1.0f);

/* USER CODE END 2 */
```

> `DAC8568_Init()` 必须放在 `MX_GPIO_Init()` 之后，否则 GPIO 端口时钟和引脚模式尚未初始化。

---

## CubeMX 重新生成代码时的注意事项

CubeMX 负责生成：

```text
main.h 中的引脚宏
gpio.c 中的 GPIO 初始化
MX_GPIO_Init()
```

用户驱动负责：

```text
DAC8568 命令格式
GPIO 模拟串行时序
DAC 初始化
电压和码值写入
```

建议把以下代码保留在 `USER CODE` 区域：

```c
#include "bsp_DAC8568.h"
volatile uint16_t g_dac8568_code[8] = {0};

DAC8568_Init();
DAC8568_SetVoltage(OutA, 1.5f);
DAC8568_SetVoltage(OutC, 1.0f);
```

`USER/bsp_DAC8568.c` 和 `USER/bsp_DAC8568.h` 不由 CubeMX 自动生成，重新 Generate Code 时通常不会被覆盖。

这份工程的核心方式是：**CubeMX 只配置三个 GPIO，DAC8568 驱动通过软件模拟串行时序完成初始化和 8 通道电压输出**。
