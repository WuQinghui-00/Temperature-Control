#include "ads1255.h"


static void delay_us(uint32_t us)
{
    volatile uint32_t count = us * (168 / 7);
    while(count--);
}
//GPIO初始化
void ADS1256_GPIO_Init(void)
{


    ADS_CS_H;
    ADS_SCLK_L;
    ADS_DIN_H;
}


//IO初始化
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
        ADS_SCLK_L; 
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

//写命令
void ADS1256_WriteCmd(uint8_t cmd)
{
    ADS_CS_L;
    ADS1256_Send8Bit(cmd);
    ADS_CS_H;
}

//写寄存器
void ADS1256_WriteReg(uint8_t reg, uint8_t value)
{
    ADS_CS_L;
    ADS1256_Send8Bit(CMD_WREG | reg);
    ADS1256_Send8Bit(0x00);  /* ?1???? */
    ADS1256_Send8Bit(value);
    ADS_CS_H;
}

//设置差分通道
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

//设置单端通道
void ADS1256_SetSingleChannel(uint8_t positive)
{
    ADS1256_SetDiffChannel(positive, NEGTIVE_AINCOM);
}


//读寄存器
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

//等待DRDY函数
uint8_t ADS1256_WaitDRDY(uint32_t timeout)
{
    while (!ADS_DRDY_IS_LOW())
    {
        if (--timeout == 0)
        {
            return 1;
        }
    }

    return 0;
}

//采样率转为SPS
double ADS1256_DataRateCodeToSPS(uint8_t drate)
{
    switch (drate)
    {
        case DATARATE_30K:    return 30000.0;
        case DATARATE_15K:    return 15000.0;
        case DATARATE_7_5K:   return 7500.0;
        case DATARATE_3_7_5K: return 3750.0;
        case DATARATE_2K:     return 2000.0;
        case DATARATE_1K:     return 1000.0;
        case DATARATE_500:    return 500.0;
        case DATARATE_100:    return 100.0;
        case DATARATE_60:     return 60.0;
        case DATARATE_50:     return 50.0;
        case DATARATE_30:     return 30.0;
        case DATARATE_25:     return 25.0;
        case DATARATE_15:     return 15.0;
        case DATARATE_10:     return 10.0;
        case DATARATE_5:      return 5.0;
        case DATARATE_2_5:    return 2.5;
        default:              return 0.0;
    }
}


//ADC初始化
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

//得到有符号32位码值
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

    // 24 位补码符号扩展到 32位 ?
    if (read & 0x800000)
    {
        read |= 0xFF000000;
    }

    return (int32_t)read;
}

//滤波函数（取最中间两个的平均值）
int32_t ADS1256_GetAdcTrimmedMean(uint16_t n)
{
    int32_t buf[ADS1256_FILTER_MAX_N];
    int32_t value;
    int32_t temp;

    if ((n < 2U) || ((n % 2U) != 0U) || (n > ADS1256_FILTER_MAX_N))
    {
        return ADS1256_INVALID_CODE;
    }

    for (uint16_t i = 0; i < n; i++)
    {
        value = ADS1256_GetAdcSigned();

        if (value == ADS1256_INVALID_CODE)
        {
            return ADS1256_INVALID_CODE;
        }

        buf[i] = value;
    }

    /* ?????? */
    for (uint16_t i = 0; i < n - 1U; i++)
    {
        for (uint16_t j = 0; j < n - 1U - i; j++)
        {
            if (buf[j] > buf[j + 1U])
            {
                temp = buf[j];
                buf[j] = buf[j + 1U];
                buf[j + 1U] = temp;
            }
        }
    }

    /* n ???,?????????? */
    return (int32_t)(((int64_t)buf[n / 2U - 1U] + (int64_t)buf[n / 2U]) / 2);
}

//滤波函数（去掉极值，取平均值）
int32_t ADS1256_GetAdcTrimmedMean1(uint16_t n)
{
    int32_t buf[ADS1256_FILTER_MAX_N];
    int32_t value;
    int32_t temp;

    if ((n < 2U) || ((n % 2U) != 0U) || (n > ADS1256_FILTER_MAX_N))
    {
        return ADS1256_INVALID_CODE;
    }

    for (uint16_t i = 0; i < n; i++)
    {
        value = ADS1256_GetAdcSigned();

        if (value == ADS1256_INVALID_CODE)
        {
            return ADS1256_INVALID_CODE;
        }

        buf[i] = value;
    }

    /* ?????? */
    for (uint16_t i = 0; i < n - 1U; i++)
    {
        for (uint16_t j = 0; j < n - 1U - i; j++)
        {
            if (buf[j] > buf[j + 1U])
            {
                temp = buf[j];
                buf[j] = buf[j + 1U];
                buf[j + 1U] = temp;
            }
        }
    }

    /* n ???,?????????? */
    return (int32_t)(((int64_t)buf[n / 2U - 1U] + (int64_t)buf[n / 2U]) / 2);
}

//将码值转换为电压
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



