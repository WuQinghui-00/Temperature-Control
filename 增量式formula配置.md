

##  增量式 PID 原理

设：

setValue = 目标温度  
actualValue = 当前实际温度  
Error =setValue - actualValue

增量式 PID 计算本次需要在原输出基础上增加或减少

当前工程对应代码：

P_Term = pid->Tec_Kp *  
         (Error - pid->LastError);  

I_Term = pid->Tec_Ki * Error;  

D_Term = pid->Tec_Kd *  
         (Error -  
          2.0f * pid->LastError +  
          pid->PrevError);  

pid->PIDInc = P_Term + I_Term + D_Term;  

pid->Tec_Current_SetPoint += pid->PIDInc;

其中：

|变量|含义|
|---|---|
|`Error`|当前误差 `e(k)`|
|`LastError`|上一次误差 `e(k-1)`|
|`PrevError`|上上次误差 `e(k-2)`|
|`PIDInc`|本周期输出增量 `Δu(k)`|
|`Tec_Current_SetPoint`|累加后的控制输出 `u(k)`|

---

##  P、I、D 三项分别做什么

### 比例增量 P

P = Kp × [e(k) - e(k-1)]

它关注误差变化量。

- 误差变化很快，P 项较大；
  
- 加快系统响应；
  
- 太大时容易振荡。
  

### 积分增量 I

I = Ki × e(k)

它关注当前是否仍有误差。

- 只要还有稳态误差，就不断修正输出；
  
- 用于消除长期偏差；
  
- 太大时会增加超调和低频振荡。
  

### 微分增量 D

D = Kd × [e(k) - 2e(k-1) + e(k-2)]

它关注误差变化趋势的变化。

- 对快速变化进行抑制；
  
- 减小超调；
  
- 对测量噪声较敏感。
  

---

##  当前代码中的小误差处理

当前 `formula.c` 在接近目标温度时进行了两层处理（误差和增量）。

###  小误差滤波

当：

|Error| < 0.05 °C

执行误差融合：

Error = Error * 0.7f  
      + pid->LastError * 0.3f;

作用是减少温度测量微小波动造成的 DAC 抖动。

### 分段减小 PID 增量

|Error| > 0.05  
    使用完整 P + I + D  

0.02 < |Error| ≤ 0.05  
    总增量乘 0.5  

|Error| ≤ 0.02  
    P 项先减半，最终总增量再乘 0.5

对应代码：

if (fabsf(Error) > 0.05f)  
{  
    pid->PIDInc =  
        P_Term + I_Term + D_Term;  
}  
else if (fabsf(Error) > 0.02f)  
{  
    pid->PIDInc =  
        (P_Term + I_Term + D_Term) * 0.5f;  
}  
else  
{  
    pid->PIDInc =  
        (P_Term * 0.5f +  
         I_Term +  
         D_Term) * 0.5f;  
}

这是为了使温度接近目标后输出更柔和。

---
## 增量式 PID 得到等效温度换算成电压

pid计算完成后：

pid->Tec_Current_SetPoint += pid->PIDInc;

再把等效温度控制量换算成 DAC 电压：

output_voltage =  
    TEC_TempV(pid->Tec_Current_SetPoint);  
    
### TEC_TempV() 做了什么：

```
函数先通过 B 值公式计算目标温度下的 NTC 电阻：

rt = FORMULA_NTC_R25 *
     expf(FORMULA_NTC_BETA *
     ((1.0f / temp_k) -
      (1.0f / FORMULA_NTC_T25_K)));


再按照左侧支路计算节点 N1 的分压：

ntc_voltage =
    FORMULA_NTC_VCC * rt /
    (FORMULA_NTC_FIXED_R + rt);
```

CTRL_TempVout(TEC_CHANNEL_1,  
              output_voltage);

### CTRL_TempVout() 中：

case TEC_CHANNEL_1:  
{  
    DAC8568_SetVoltage(OutA, voltage);  
    break;  
}

所以 TEC1 的实际输出链路是：

温度误差  
  ↓  
增量式 PID  
  ↓  
Tec_Current_SetPoint  
  ↓  
TEC_TempV()  
  ↓  
DAC8568 OutA  
  ↓  
DAC0

---

## 源代码
### formula.h
```
#ifndef __FORMULA_H
#define __FORMULA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * TEC ????
 * ????????? Tec_Channel ??,??????,
 * ??????????????
 */
typedef enum
{
    TEC_CHANNEL_1 = 0,
    TEC_CHANNEL_2 = 1
} Tec_Channel;



typedef struct
{
    float Tec_Kp;
    float Tec_Ki;
    float Tec_Kd;

    float LastError;
    float PrevError;

    float PIDInc;

    float Tec_Current_SetPoint;

} PID;



/* PID ³õÊ¼»¯ */
void Formula_TecPid_Init(PID *pid,
                         float kp,
                         float ki,
                         float kd,
                         float init_setpoint);

/*
 * PID ×´Ì¬ÇåÁã
 *
 */
void Formula_TecPid_Reset(PID *pid);


void Formula_TecTemperature_PID(float setValue,
                                float actualValue,
                                PID *pid,
                                Tec_Channel tec_channel);

											
															
/* ÎÂ¶È·´Ëã³É NTC µçÑ¹ */
float TEC_TempV(float Tec_Temp);
										
//Êä³öTEC¿ØÖÆµçÑ¹
void CTRL_TempVout(Tec_Channel tec_channel, float voltage);															 
															 
#ifdef __cplusplus
}
#endif

#endif



```
### formula.c
```
#include "formula.h"
#include "bsp_DAC8568.h"
#include <math.h>

/*
 * ?????????????
 *
 * TEC_TempV(temp):
 *      ????????? NTC ?????
 *
 * CTRL_TempVout(channel, voltage):
 *      ????????? TEC ?????
 *
 * ???????????????????,
 * ?????? extern ??,?? include ??????
 */
extern float TEC_TempV(float Tec_Temp);
extern void CTRL_TempVout(Tec_Channel tec_channel, float voltage);


//NTC²ÎÊý
#define FORMULA_NTC_VCC              2.5f
#define FORMULA_NTC_FIXED_R          10000.0f
#define FORMULA_NTC_R25              10000.0f
#define FORMULA_NTC_BETA             3950.0f
#define FORMULA_NTC_T25_K            298.15f
/*
 * PID Êä³öÏÞ·ù
 *
 */
#define FORMULA_TEC_PID_OUT_MIN      0.0f
#define FORMULA_TEC_PID_OUT_MAX      300.0f


/*
 * Îó²î·Ö¶ÎãÐÖµ
 *
 * ?????:
 *
 * fabs(Error) > 0.05:
 *      PIDInc = P + I + D
 *
 * 0.02 < fabs(Error) <= 0.05:
 *      PIDInc = (P + I + D) * 0.5
 *
 * fabs(Error) <= 0.02:
 *      PIDInc = (P * 0.5 + I + D) * 0.5
 */
#define FORMULA_TEC_ERR_BIG          0.05f
#define FORMULA_TEC_ERR_SMALL        0.02f


/*
 * Ð¡Îó²îÆ½»¬ÏµÊý£¬Îó²îÈÚºÏ
 *
 *
 * if(fabs(Error) < 0.05)
 * {
 *     Error = Error * 0.7 + LastError * 0.3;
 * }
 */
#define FORMULA_TEC_ERR_FILTER_NOW   0.7f
#define FORMULA_TEC_ERR_FILTER_LAST  0.3f


static float Formula_LimitFloat(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}


/*
 * PID ³õÊ¼»¯
 */
void Formula_TecPid_Init(PID *pid,
                         float kp,
                         float ki,
                         float kd,
                         float init_setpoint)
{
    if (pid == 0)
    {
        return;
    }

    pid->Tec_Kp = kp;
    pid->Tec_Ki = ki;
    pid->Tec_Kd = kd;

    pid->LastError = 0.0f;
    pid->PrevError = 0.0f;
    pid->PIDInc = 0.0f;

    pid->Tec_Current_SetPoint =
        Formula_LimitFloat(init_setpoint,
                           FORMULA_TEC_PID_OUT_MIN,
                           FORMULA_TEC_PID_OUT_MAX);
}


/*
 * PID ×´Ì¬ÇåÁã
 */
void Formula_TecPid_Reset(PID *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->LastError = 0.0f;
    pid->PrevError = 0.0f;
    pid->PIDInc = 0.0f;
}

/*
 * ÎÂ¶È -> NTC ½ÚµãµçÑ¹
 *
 * ??:
 *      PID ???????????? Tec_Current_SetPoint?
 *      ?????????? NTC ??????
 *
 * ¹«Ê½:
 *      Rt = R25 * exp(B * (1/T - 1/T25))
 *      Vn1 = VCC * Rt / (R_FIXED + Rt)
 */
float TEC_TempV(float Tec_Temp)
{
    float temp_k;
    float rt;
    float ntc_voltage;

    temp_k = Tec_Temp + 273.15f;

    if (temp_k <= 0.0f)
    {
        return 0.0f;
    }

    rt = FORMULA_NTC_R25 *
         expf(FORMULA_NTC_BETA *
         ((1.0f / temp_k) - (1.0f / FORMULA_NTC_T25_K)));

    ntc_voltage = FORMULA_NTC_VCC * rt /
                  (FORMULA_NTC_FIXED_R + rt);

    ntc_voltage = Formula_LimitFloat(ntc_voltage, 0.0f, FORMULA_NTC_VCC);

    return ntc_voltage;
}


/*
 * TEC ¿ØÖÆµçÑ¹Êä³ö
 *
 * ??? TEC ????? DAC8568 ???
 *
 * ????:
 *      TEC_1 -> DAC8568 OutA
 *      TEC_2 -> DAC8568 OutC
 *
 * ????????????,????????
 */
void CTRL_TempVout(Tec_Channel tec_channel, float voltage)
{
    voltage = Formula_LimitFloat(voltage, 0.0f, 5.0f);

    switch (tec_channel)
    {
        case TEC_CHANNEL_1:
        {
            DAC8568_SetVoltage(OutA, voltage);
            break;
        }

        case TEC_CHANNEL_2:
        {
            DAC8568_SetVoltage(OutC, voltage);
            break;
        }

        default:
        {
            break;
        }
    }
}


/*
 * ÉèÖÃPID ²ÎÊý
 */
void TEC_PID_SetParam(PID *pid,
                      float kp,
                      float ki,
                      float kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->Tec_Kp = kp;
    pid->Tec_Ki = ki;
    pid->Tec_Kd = kd;
}


/*
 * PID ×´Ì¬¸´Î»
 */
void TEC_PID_Reset(PID *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->LastError = 0.0f;
    pid->PrevError = 0.0f;
    pid->PIDInc = 0.0f;
}


/*
 * TECÎÂ¶È¼ÆËã
 *
 */
void Formula_TecTemperature_PID(float setValue,
                                float actualValue,
                                PID *pid,
                                Tec_Channel tec_channel)
{
    float Error;
    float P_Term;
    float I_Term;
    float D_Term;
    float output_voltage;

    if (pid == 0)
    {
        return;
    }

    /*
     * 1. ¼ÆËãµ±Ç°Îó²î
     *
     * error > 0:
     *      Ä¿±ê±ÈÊµ¼Ê¸ß£¬ÐèÒªÉýÎÂ
     *
     * error < 0:
     *      Ä¿±ê±ÈÊµ¼ÊµÍ£¬½µÎÂ
     */
    Error = setValue - actualValue;

    /*
     * 2. Ð¡Îó²îÇøÓòÆ½»¬
     *
     *Îó²îºÜÐ¡Ê±£¬Ö±½ÓÊ¹ÓÃµ±Ç°Îó²î¿ÉÄÜ»áµ¼ÖÂÊä³ö¶¶¶¯
		 *ËùÒÔÕâÀï°Ñµ±Ç°Îó²îºÍÉÏÒ»´ÎÎó²î×ö¼ÓÈ¨Æ½¾ù¿ÉÒÔ
     */
if (fabsf(Error) < FORMULA_TEC_ERR_BIG)
    {
        Error = Error * FORMULA_TEC_ERR_FILTER_NOW
              + pid->LastError * FORMULA_TEC_ERR_FILTER_LAST;
    }


    /*
     * 3. PID¼ÆËã
     *
     */
    P_Term = pid->Tec_Kp * (Error - pid->LastError);//Kp * (±¾´ÎÎó²î-ÉÏ´ÎÎó²î)£¬Îó²î±ä»¯¿ì£¬p¾Í´ó


    I_Term = pid->Tec_Ki * Error;  //¸úµ±Ç°Îó²îÓÐ¹Ø£¬ÓÐÎó²î¾ÍÍùÊä³öÀï¼ÓÒ»µãµã£¬Ïû³ýÎÈÌ¬Îó²î

     D_Term = pid->Tec_Kd *
             (Error - 2.0f * pid->LastError + pid->PrevError);  //Îó²î±ä»¯Ì«´ó£¬D»áÒÖÖÆÊä³ö£¬¼õÉÙ¹ý³å

    /*
     * 4. ¸ù¾ÝÎó²î´óÐ¡µ÷ÕûpidÔöÁ¿
     *
     * ´ó:
     *      Õý³£Êä³öPIDÔöÁ¿
     *
     * ÖÐµÈ:
     *      PIDÔöÁ¿¼õ°ë
     *
     * Ð¡:
     *      PÏî¼õ°ë£¬×ÜÊä³öÔÙ¼õ°ë
     */
    if (fabsf(Error) > FORMULA_TEC_ERR_BIG)
    {
        pid->PIDInc = P_Term + I_Term + D_Term;
    }
    else if (fabsf(Error) > FORMULA_TEC_ERR_SMALL)
    {
        pid->PIDInc = (P_Term + I_Term + D_Term) * 0.5f;
    }
    else
    {
        pid->PIDInc = (P_Term * 0.5f + I_Term + D_Term) * 0.5f;
    }

    /*
     * 5. ÀÛ¼ÓPIDÔöÁ¿
     *
     * ??:
     * ???????????,?????
     */
    pid->Tec_Current_SetPoint += pid->PIDInc; 

    /*
     * 6. ¸üÐÂÀúÊ·Îó²î
     */
    pid->PrevError = pid->LastError;
    pid->LastError = Error;

    /*
     * 7. Êä³öÏÞ·ù
     *
     * ???????? 0~300?
     */
pid->Tec_Current_SetPoint =
        Formula_LimitFloat(pid->Tec_Current_SetPoint,
                           FORMULA_TEC_PID_OUT_MIN,
                           FORMULA_TEC_PID_OUT_MAX);

    /*
     * 8. ??????????,????
     *
     * TEC_TempV():
     *      ?? -> NTC ????
     *
     * CTRL_TempVout():
     *      ??????? TEC ??
     */
    output_voltage = TEC_TempV(pid->Tec_Current_SetPoint);

    CTRL_TempVout(tec_channel, output_voltage);

}

```

## 在main.c 中调用 

在调用formula 函数时，应显式包含：

#include "formula.h"

建议放在：

/* USER CODE BEGIN Includes */  
#include "formula.h"  
​  
/* USER CODE END Includes */

这样可以避免函数隐式声明和参数类型不一致。

 全局对象中定义：
 /* USER CODE BEGIN PV */

PID TEC1_PID;

/* USER CODE END PV */

---

##  初始化代码

在外设初始化、DAC 初始化完成后：

Formula_TecPid_Init(  
    &TEC1_PID,  
    0.8f,                       /* 初始 Kp */  
    0.02f,                      /*初始 Ki */  
    0.1f,                       /* 初始 Kd */  
    TEC1_TARGET_TEMPERATURE  
);  


---

## 主循环调用流程

必须先完成：

ADS1256 读取  
    ↓  
电压换算  
    ↓  
NTC 温度计算  
    ↓  
得到 ntc_temperature 和 ntc_data_valid

然后执行控制任务。



---

##  建议串口打印的状态


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
