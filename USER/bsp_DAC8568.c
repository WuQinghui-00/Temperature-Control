#include "bsp_DAC8568.h"

volatile uint16_t g_dac8568_code[8] = {0U};


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
void DAC8568_SetVoltage(
    uint8_t channel,
    double voltage_v)
{
    uint32_t code_u32;
    uint16_t dac_code;

    if (channel > 7U)
    {
        return;
    }

    if (voltage_v <= 0.0)
    {
        dac_code = 0U;
    }
    else if (voltage_v >= 5.0)
    {
        dac_code = 65535U;
    }
    else
    {
        code_u32 =
            (uint32_t)(
                voltage_v *
                65535.0 /
                5.0 +
                0.5
            );

        dac_code =
            (uint16_t)code_u32;
    }

    DAC8568_Write_passageway(
        channel,
        dac_code
    );

    g_dac8568_code[channel] =
        dac_code;
}


/**
  * @brief  设置DAC8568通道 电压
  * @param  mCH:通道号0-7
  * @param  DAC_Set：设置的DAC值
  */
void DAC8568_SetDAC(uint8_t channel, int32_t dac_set)
{
    uint16_t dac_code;

    if (channel > 7U)
    {
        return;
    }

    if (dac_set <= 0)
    {
        dac_code = 0U;
    }
    else if (dac_set >= 65535)
    {
        dac_code = 65535U;
    }
    else
    {
        dac_code = (uint16_t)dac_set;
    }

    DAC8568_SYNC_H;
    Mcpdely();

    DAC8568_Write_passageway(channel, dac_code);

    g_dac8568_code[channel] = dac_code;
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

