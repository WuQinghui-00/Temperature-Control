# Thermal 模块配置

---

## 1. NTC 的基本原理

NTC 是负温度系数热敏电阻：

```text
温度升高  -> NTC 电阻减小
温度降低  -> NTC 电阻增大
```

当前使用的是 10 kΩ、B=3950 的 NTC。

在 25℃ 时：

```text
RNTC = 10 kΩ
```

温度高于 25℃ 时：

```text
RNTC < 10 kΩ
```

温度低于 25℃ 时：

```text
RNTC > 10 kΩ
```

负温度是正常有效结果，不需要对温度结果做限零处理。

---

## 2. 当前电桥结构

当前 `thermal.h` 中定义的电桥如下：

```text
                         2.5 V
                           |
               +-----------+-----------+
               |                       |
             10 kΩ                   10 kΩ
               |                       |
              N1                      N2
               |                       |
            25℃/10 kΩ NTC            10 kΩ
               |                       |
              GND                     GND
```

ADS1256 差分输入定义为：

```text
Vdiff = V(N1) - V(N2)
```

其中：

- `N1` 是 NTC 测量节点
- `N2` 是固定参考节点
- ADS1256 的 AIN0 接 N1
- ADS1256 的 AIN1 接 N2

---

## 3. 参考节点 N2 的计算

参考支路是两个 10 kΩ 电阻分压：

```text
2.5 V -- 10 kΩ -- N2 -- 10 kΩ -- GND
```

对应 `thermal.c` 代码：

```c
static double NTC_GetReferenceNodeVoltage(void)
{
    double denominator;

    denominator =
        NTC_REF_TOP_OHM +
        NTC_REF_BOTTOM_OHM;

    if (denominator <= 0.0)
    {
        return 0.0;
    }

    return
        NTC_BRIDGE_VOLTAGE_V *
        NTC_REF_BOTTOM_OHM /
        denominator;
}
```

因此在当前配置下：

```text
V(N2) = 1.25 V
```

---

## 4. 差分电压还原 NTC 节点电压

ADS1256 测量的是：差分电压Vdiff

```c
reference_node_voltage =
    NTC_GetReferenceNodeVoltage();

ntc_node_voltage =
    diff_voltage +
    reference_node_voltage;
```

例如 ADS1256 测得：

```text
Vdiff = +0.677 V
```

则NTC节点N1电压为：

```text
VN1 = 0.677 + 1.25
    = 1.927 V
```

该节点电压大于 1.25 V，说明 NTC 电阻大于 10 kΩ，对应低于 25℃。

---

## 5. N1 电压换算成 NTC 电阻

左侧电路是：

```text
2.5 V -- Rfixed -- N1 -- RNTC -- GND
```

分压公式：

VN1= 2.5 × RNTC / (Rfixed + RNTC)

故反推可得：

RNTC = Rfixed × VN1/ (2.5 - VN1)

对应代码：

```c
ntc_resistance =
    NTC_FIXED_OHM *
    ntc_node_voltage /
    (NTC_BRIDGE_VOLTAGE_V -
     ntc_node_voltage);
```

---

## 6. NTC 电阻换算成温度

当前程序使用 B 值公式：

Temp = 1 / ( ln(RNTC/Rp)/Bx + 1/T2 ) - 273.15

其中：

RtNTC: 当前 NTC 阻值
Rp : 25℃ 时的 NTC 标称阻值，一般是 10000Ω
Bx : NTC 的 B 值，比如 3950
T2 : 标称温度对应的开尔文温度，一般是 298.15K

对应 `thermal.c` 代码：

```c
nominal_temperature_k =
    NTC_NOMINAL_TEMP_C +
    NTC_KELVIN_OFFSET;

inverse_temperature =
    (1.0 / nominal_temperature_k) +
    (
        log(
            ntc_resistance /
            NTC_R25_OHM
        )
        /
        NTC_BETA_VALUE
    );

temperature_k =
    1.0 /
    inverse_temperature;

*temperature_c =
    temperature_k -
    NTC_KELVIN_OFFSET;
```

---

## 10. 完整的“电压到温度”流程

`NTC_DiffVoltageToTemperature()` 的完整计算流程是：

```text
ADS1256 原始码
       |
       v
ADS1256_CodeToVoltage()
       |
       v
差分电压 Vdiff
       |
       v
VN2 = 参考支路分压
       |
       v
VN1 = Vdiff + VN2
       |
       v
RNTC = Rfixed × VN1/ (2.5 - VN1)
       |
       v
B 值公式计算温度 K
       |
       v
温度 ℃ = 温度 K - 273.15
```

函数声明：

```c
int NTC_DiffVoltageToTemperature(double diff_voltage,
                                 double *temperature_c,
                                 double *resistance_ohm);
```

---

## 在`main.c` 中调用

`main.c` 调用 Thermal 函数时，应显式包含：

```c
#include "thermal.h"
```

建议放在：

```c
/* USER CODE BEGIN Includes */
#include "thermal.h"

/* USER CODE END Includes */
```

这样可以避免函数隐式声明和参数类型不一致。

---

## 1. `main.c` 中的变量配置

当前主要温度变量：

```c
static int32_t ntc_adc_code = 0;
static double ntc_diff_voltage = 0.0;
static double ntc_temperature = 0.0;
static double ntc_resistance = 0.0;

static double dac_temperature = 0.0;
static double dac_ntc_resistance = 0.0;
static uint8_t dac_temp_valid = 0U;

static uint8_t ntc_data_valid = 0U;
static uint32_t ntc_raw24 = 0U;
```

建议全部使用：

```c
0.0
```

而不是：

```c
0.0f
```

因为变量类型是 `double`。

---

## 2. `main.c` 中的初始化流程

当前初始化代码：

```c
UART1_Comm_Start();

DAC8568_Init();

/* DAC0 -> AIN0 */
DAC8568_SetVoltage(OutA, 1.25);

/* DAC2 -> AIN1 */
DAC8568_SetVoltage(OutC, 1.25);

ADS1256_GPIO_Init();
ADS1256_CfgADC(PGA_1, DATARATE_50);

/* AIN0 - AIN1 */
ADS1256_SetDiffChannel(
    POSITIVE_AIN0,
    NEGTIVE_AIN1
);
```

初始化时：

```text
DAC0 = 1.25 V
DAC2 = 1.25 V
Vdiff = 0 V
```

按照当前 10 kΩ NTC 模型，对应约 25℃。

---

## 3. `main.c` 中 ADC 读取流程

### 读取滤波后的 ADC 原始码

```c
ntc_adc_code =
    ADS1256_GetAdcTrimmedMean1(ADC_FILTER_N);
```

当前配置：

```c
#define ADC_FILTER_N 10U
```

表示每次使用 10 个样本进行处理。

### 读取流程

```c
ntc_data_valid = 0U;
ntc_raw24 = 0U;
    
    //判断adc返回值
    if (ntc_adc_code != ADS1256_INVALID_CODE)
{
    ntc_raw24 =
        ((uint32_t)ntc_adc_code) &
        0x00FFFFFFU;

	  //adc测量差分电压
    ntc_diff_voltage =
            ADS1256_CodeToVoltage(
            ntc_adc_code,
            2.5,
            PGA_1
        );

	  //还原节点N1电压
	  adc_ntc_node_voltage =
            ntc_diff_voltage +
            dac2_v;
	
						
	//差分电压转换成温度和电阻
	ntc_data_valid =
    NTC_NodeVoltageToTemperature(
        adc_ntc_node_voltage,
        &ntc_temperature,
        &ntc_resistance
    );

  }
```

##  `main.c` 中温度计算调用

根据理论差分电压计算模拟温度：

当前调用：

```c

		 dac_temp_valid =
    NTC_NodeVoltageToTemperature(
        dac0_v,
        &dac_temperature,
        &dac_ntc_resistance
    );
```

---

## 4 串口输出温度

当前串口格式：

```c
    if ((HAL_GetTick() - last_send_tick) >= 500U)
    {
        last_send_tick = HAL_GetTick();

        if (!UART1_Comm_IsTxBusy())
        {if ((ntc_data_valid != 0U) &&
                (dac_temp_valid != 0U))
            {
                (void)UART1_SendString_DMA(
    "TEMP=%.7fC,"
    "ADC_DIFF=%.7fV,"
    "DAC_DIFF=%.7fV,"
    "DAC0_V=%.5fV,"
    "DAC2_V=%.5fV,"
    "RAW24=0x%06lX,"
    "CODE32=%ld\r\n",

    ntc_temperature,
		ntc_diff_voltage,
    dac_diff_v,
    dac0_v,
    dac2_v,
    (unsigned long)ntc_raw24,
    (long)ntc_adc_code
);
            }
            else
            {
                (void)UART1_SendString_DMA(
                    "TEMP=INVALID,"
                    "ADC_DIFF=%.7fV,"
                    "DAC_DIFF=%.7fV,"
                    "DAC0_V=%.5fV,"
                    "DAC2_V=%.5fV,"
                    "RAW24=0x%06lX,"
                    "CODE32=%ld\r\n",
								
                    ntc_diff_voltage,
                    dac_diff_v,
                    dac0_v,
                    dac2_v,

                    (unsigned long)ntc_raw24,
                    (long)ntc_adc_code
                );
            }
        }
				
    }
```

其中：

```c
TEMP=%.7fC
```

表示温度显示 7 位小数。

例如：

```text
TEMP=24.9958525C,ADC_DIFF=1.2501152V,DAC_DIFF=1.2500191V,DAC0_V=1.25002V,DAC2_V=0.00000V,RAW24=0x2000C1,CODE32=2097345
TEMP=24.9957667C,ADC_DIFF=1.2501176V,DAC_DIFF=1.2500191V,DAC0_V=1.25002V,DAC2_V=0.00000V,RAW24=0x2000C5,CODE32=2097349
TEMP=24.9959383C,ADC_DIFF=1.2501128V,DAC_DIFF=1.2500191V,DAC0_V=1.25002V,DAC2_V=0.00000V,RAW24=0x2000BD,CODE32=2097341
```

负温度会由格式化函数自动显示负号，不需要额外处理。

---

