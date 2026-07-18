# 自动整定 PID 与增量式 PID 使用说明

> 适用工程：STM32F407 + ADS1256 + DAC8568 + TEC 温控 当前目标温度示例：`25.0 °C` 当前控制周期示例：`200 ms`

---

## 重要原理

自动整定 PID 和增量式 PID **不是互相替代的两套控制方法**，而是前后衔接的两个阶段：

自动整定阶段  
    ↓  
让温度在目标值附近产生受控振荡  
    ↓  
测量振荡幅值和周期  
    ↓  
计算得到合适的pid参数 Kp、Ki、Kd  
    ↓  
把参数写入 PID 结构体  
    ↓  
增量式 PID 接管  
    ↓  
持续根据实际温度修正 DAC0 输出
    

---


# 第一部分：自动整定 PID

##  自动整定的基本原理

当前 `pid.c` 使用的是 **继电反馈自动整定法**。

设目标温度为：

Tset = 25.0 °C

默认继电步幅为：

config->target_temperature = 25.0f;
config->relay_step = 2.0f;


于是整定时使用两个控制档位：

高档输出 = 25 + 2 = 27  
低档输出 = 25 - 2 = 23
```
output_high = 25.0f + 2.0f;   /* 高档：27 */
output_low  = 25.0f - 2.0f;   /* 低档：23 */

```

这里的 `27` 和 `23` 是：

> 传给 `TEC_TempV()` 的等效温度控制量。目的是为了让温度形成周期振荡，用峰值幅度周期等值从而计算出合适的PID参数。

---

##  PID 参数在哪里写入增量式 PID

真正把自动整定结果反馈给普通 PID 的位置是：

int PID_AutoTune_ApplyResult(const PID_AutoTune *tuner,  
                             PID *pid)  
{  
    pid->Tec_Kp = tuner->result_kp;  
    pid->Tec_Ki = tuner->result_ki;  
    pid->Tec_Kd = tuner->result_kd;  

    Formula_TecPid_Reset(pid);  
      
    return 1;  
}


所以整定成功后，不需要在 `main.c` 再手动赋值：

TEC1_PID.Tec_Kp = ...  
TEC1_PID.Tec_Ki = ...  
TEC1_PID.Tec_Kd = ...

参数已经自动写入 `TEC1_PID`。


自动写入的完整数据流是：

PID_AutoTune_CalculateGains()  
        ↓  
tuner->result_kp  
tuner->result_ki  
tuner->result_kd  
        ↓  
PID_AutoTune_ApplyResult()  
        ↓  
TEC1_PID.Tec_Kp  
TEC1_PID.Tec_Ki  
TEC1_PID.Tec_Kd  
        ↓  
Formula_TecTemperature_PID()

---
## 源代码
### pid.h
```
#ifndef __PID_AUTOTUNE_H
#define __PID_AUTOTUNE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "formula.h"

/*
 * pid.h / pid.c
 *
 * TEC PID relay auto-tuner for this project.
 *
 * Important:
 * 1. The existing Formula_TecTemperature_PID() is an incremental PID:
 *
 *      du[k] = Kp * (e[k] - e[k-1])
 *            + Ki * e[k]
 *            + Kd * (e[k] - 2e[k-1] + e[k-2])
 *
 * 2. Therefore the tuner converts continuous tuning results to the
 *    discrete Kp/Ki/Kd representation used by the project.
 *
 * 3. PID_AutoTune_Task() is non-blocking. Call it repeatedly and pass
 *    HAL_GetTick() as now_ms.
 *
 * 4. During auto-tuning, do not call Formula_TecTemperature_PID() for
 *    the same TEC channel, because the tuner must exclusively control
 *    Tec_Current_SetPoint.
 *
 * 5. After tuning, run the normal PID at the same period configured by
 *    sample_time_ms, otherwise Ki/Kd will no longer match the real period.
 */

#define PID_AUTOTUNE_MAX_CYCLES       (8U)

typedef enum
{
    PID_AUTOTUNE_RULE_ZN_PI = 0,
    PID_AUTOTUNE_RULE_ZN_PID,
    PID_AUTOTUNE_RULE_TYREUS_LUYBEN_PID
} PID_AutoTuneRule;

typedef enum
{
    PID_AUTOTUNE_STATE_IDLE = 0,
    PID_AUTOTUNE_STATE_RELAY_HIGH,
    PID_AUTOTUNE_STATE_RELAY_LOW,
    PID_AUTOTUNE_STATE_CALCULATING,
    PID_AUTOTUNE_STATE_FINISHED,
    PID_AUTOTUNE_STATE_ABORTED,
    PID_AUTOTUNE_STATE_ERROR
} PID_AutoTuneState;

typedef enum
{
    PID_AUTOTUNE_ERROR_NONE = 0,
    PID_AUTOTUNE_ERROR_BAD_ARGUMENT,
    PID_AUTOTUNE_ERROR_BAD_CONFIG,
    PID_AUTOTUNE_ERROR_INVALID_TEMPERATURE,
    PID_AUTOTUNE_ERROR_SAFETY_LIMIT,
    PID_AUTOTUNE_ERROR_TOTAL_TIMEOUT,
    PID_AUTOTUNE_ERROR_HALF_CYCLE_TIMEOUT,
    PID_AUTOTUNE_ERROR_OSCILLATION_TOO_SMALL,
    PID_AUTOTUNE_ERROR_OSCILLATION_UNSTABLE
} PID_AutoTuneError;

typedef struct
{
    /*
     * Actual temperature around which relay oscillation is generated.
     */
    float target_temperature;

    /*
     * Relay output step in the project's "equivalent temperature"
     * output unit. The tuner drives:
     *
     *     target_temperature + relay_step
     *     target_temperature - relay_step
     *
     * A recommended initial value for TEC is 1.5 to 3.0 degrees.
     */
    float relay_step;

    /*
     * Switching hysteresis around target_temperature.
     * A recommended initial value is 0.10 to 0.30 degree.
     */
    float hysteresis;

    /*
     * Allowed range of Tec_Current_SetPoint.
     * For the present temperature acquisition code, 0 to 60 degrees
     * is a conservative default.
     */
    float output_min;
    float output_max;

    /*
     * Absolute process-temperature protection limits.
     */
    float safety_temperature_min;
    float safety_temperature_max;

    /*
     * Tuner and normal PID execution period.
     */
    uint32_t sample_time_ms;

    /*
     * Maximum duration of the whole tuning operation.
     */
    uint32_t total_timeout_ms;

    /*
     * Maximum duration of one rising/falling half-cycle.
     */
    uint32_t half_cycle_timeout_ms;

    /*
     * Minimum accepted peak-to-peak process oscillation.
     */
    float minimum_peak_to_peak;

    /*
     * Maximum relative deviation allowed among measured cycles.
     * Example: 0.35 means +/-35 percent.
     */
    float stability_tolerance;

    /*
     * Number of stable full cycles needed to calculate gains.
     * Valid range: 3 to PID_AUTOTUNE_MAX_CYCLES.
     */
    uint8_t required_cycles;

    /*
     * Maximum number of measured cycles before reporting unstable.
     * Valid range: required_cycles to PID_AUTOTUNE_MAX_CYCLES.
     */
    uint8_t maximum_cycles;

    /*
     * Default recommendation:
     * PID_AUTOTUNE_RULE_TYREUS_LUYBEN_PID
     * It is less aggressive than classic Ziegler-Nichols PID.
     */
    PID_AutoTuneRule rule;

    /*
     * 1: automatically write the result into PID after success.
     * 0: keep the old gains; call PID_AutoTune_ApplyResult() manually.
     */
    uint8_t apply_result_automatically;

} PID_AutoTuneConfig;

typedef struct
{
    PID_AutoTuneConfig config;

    PID_AutoTuneState state;
    PID_AutoTuneError error;
    Tec_Channel channel;

    uint32_t start_time_ms;
    uint32_t last_sample_time_ms;
    uint32_t half_cycle_start_time_ms;
    uint32_t last_upper_cross_time_ms;

    uint8_t upper_cross_valid;
    uint8_t pending_high_peak_valid;
    uint8_t cycle_count;
    uint8_t progress;

    float output_center;
    float output_high;
    float output_low;
    float current_output;

    float half_cycle_min;
    float half_cycle_max;
    float pending_high_peak;

    float cycle_amplitude[PID_AUTOTUNE_MAX_CYCLES];
    float cycle_period_s[PID_AUTOTUNE_MAX_CYCLES];

    float ultimate_gain;
    float ultimate_period_s;

    float result_kp;
    float result_ki;
    float result_kd;

    float saved_kp;
    float saved_ki;
    float saved_kd;
    float saved_output;

} PID_AutoTune;

/*
 * Fill config with conservative defaults suitable for this TEC project.
 *
 * target_temperature:
 *     Desired tuning temperature.
 *
 * sample_time_ms:
 *     Real period of both PID_AutoTune_Task() data processing and the
 *     normal Formula_TecTemperature_PID() task after tuning.
 */
void PID_AutoTune_DefaultConfig(PID_AutoTuneConfig *config,
                                float target_temperature,
                                uint32_t sample_time_ms);

/*
 * Initialize a tuner object. Call before PID_AutoTune_Start().
 *
 * Returns:
 *     1: success
 *     0: invalid argument or configuration
 */
int PID_AutoTune_Init(PID_AutoTune *tuner,
                      const PID_AutoTuneConfig *config);

/*
 * Start relay auto-tuning.
 *
 * The present project uses a larger Tec_Current_SetPoint to request a
 * higher temperature. This function is written for that direct relation.
 *
 * Returns:
 *     1: started
 *     0: failed; inspect tuner->error
 */
int PID_AutoTune_Start(PID_AutoTune *tuner,
                       PID *pid,
                       float actual_temperature,
                       uint32_t now_ms,
                       Tec_Channel channel);

/*
 * Non-blocking tuning task.
 *
 * Call repeatedly. The function internally accepts one sample every
 * config.sample_time_ms and directly drives the selected DAC channel.
 */
PID_AutoTuneState PID_AutoTune_Task(PID_AutoTune *tuner,
                                    PID *pid,
                                    float actual_temperature,
                                    uint32_t now_ms);

/*
 * Abort tuning and restore the PID gains/output saved at Start().
 */
void PID_AutoTune_Abort(PID_AutoTune *tuner,
                        PID *pid);

/*
 * Apply a previously calculated result to the project PID structure.
 *
 * Returns:
 *     1: applied
 *     0: result not available or bad argument
 */
int PID_AutoTune_ApplyResult(const PID_AutoTune *tuner,
                             PID *pid);

/*
 * Helper query functions.
 */
int PID_AutoTune_IsRunning(const PID_AutoTune *tuner);
int PID_AutoTune_HasResult(const PID_AutoTune *tuner);
uint8_t PID_AutoTune_GetProgress(const PID_AutoTune *tuner);


/* High-level controller: keeps mode switching out of main.c. */
typedef enum
{
    PID_CONTROL_MODE_MANUAL = 0,
    PID_CONTROL_MODE_AUTOTUNE,
    PID_CONTROL_MODE_CLOSED_LOOP
} PID_ControlMode;

typedef struct
{
    PID *pid;
    PID_AutoTune autotune;
    PID_AutoTuneConfig config;

    PID_ControlMode mode;
    Tec_Channel channel;

    float target_temperature;
    uint32_t control_period_ms;
    uint32_t last_pid_time_ms;

    uint8_t autotune_request;
} PID_Control;

int PID_Control_Init(PID_Control *control,
                     PID *pid,
                     float target_temperature,
                     uint32_t control_period_ms,
                     Tec_Channel channel);

void PID_Control_RequestAutoTune(PID_Control *control);
void PID_Control_SetManual(PID_Control *control);
void PID_Control_EnablePID(PID_Control *control,
                           uint32_t now_ms);

PID_ControlMode PID_Control_Task(PID_Control *control,
                                 float actual_temperature,
                                 uint8_t temperature_valid,
                                 uint32_t now_ms);

PID_ControlMode PID_Control_GetMode(const PID_Control *control);

#ifdef __cplusplus
}
#endif

#endif
```
### pid.c
```
#include "pid.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define PID_AUTOTUNE_PI                  (3.14159265358979323846f)
#define PID_AUTOTUNE_MIN_SAMPLE_MS       (10U)
#define PID_AUTOTUNE_MIN_CYCLES          (3U)
#define PID_AUTOTUNE_EPSILON             (0.000001f)

/*
 * This module drives the same "equivalent temperature" output used by
 * Formula_TecTemperature_PID().
 */
static float PID_AutoTune_LimitFloat(float value,
                                     float minimum,
                                     float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}


static float PID_AutoTune_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}


static int PID_AutoTune_IsFiniteFloat(float value)
{
    if (value != value)
    {
        return 0;
    }

    if ((value >= FLT_MAX) || (value <= -FLT_MAX))
    {
        return 0;
    }

    return 1;
}


static uint32_t PID_AutoTune_ElapsedMs(uint32_t now_ms,
                                       uint32_t old_ms)
{
    /*
     * Unsigned subtraction is wrap-around safe for HAL_GetTick().
     */
    return now_ms - old_ms;
}


static int PID_AutoTune_ConfigIsValid(const PID_AutoTuneConfig *config)
{
    if (config == NULL)
    {
        return 0;
    }

    if (!PID_AutoTune_IsFiniteFloat(config->target_temperature) ||
        !PID_AutoTune_IsFiniteFloat(config->relay_step) ||
        !PID_AutoTune_IsFiniteFloat(config->hysteresis) ||
        !PID_AutoTune_IsFiniteFloat(config->output_min) ||
        !PID_AutoTune_IsFiniteFloat(config->output_max) ||
        !PID_AutoTune_IsFiniteFloat(config->safety_temperature_min) ||
        !PID_AutoTune_IsFiniteFloat(config->safety_temperature_max) ||
        !PID_AutoTune_IsFiniteFloat(config->minimum_peak_to_peak) ||
        !PID_AutoTune_IsFiniteFloat(config->stability_tolerance))
    {
        return 0;
    }

    if (config->relay_step <= 0.0f)
    {
        return 0;
    }

    if (config->hysteresis <= 0.0f)
    {
        return 0;
    }

    if (config->output_max <= config->output_min)
    {
        return 0;
    }

    if (config->safety_temperature_max <=
        config->safety_temperature_min)
    {
        return 0;
    }

    if ((config->target_temperature <=
         config->safety_temperature_min) ||
        (config->target_temperature >=
         config->safety_temperature_max))
    {
        return 0;
    }

    if (config->sample_time_ms < PID_AUTOTUNE_MIN_SAMPLE_MS)
    {
        return 0;
    }

    if ((config->total_timeout_ms == 0U) ||
        (config->half_cycle_timeout_ms == 0U) ||
        (config->total_timeout_ms <= config->half_cycle_timeout_ms))
    {
        return 0;
    }

    if (config->minimum_peak_to_peak <= 0.0f)
    {
        return 0;
    }

    if ((config->stability_tolerance <= 0.0f) ||
        (config->stability_tolerance > 1.0f))
    {
        return 0;
    }

    if ((config->required_cycles < PID_AUTOTUNE_MIN_CYCLES) ||
        (config->required_cycles > PID_AUTOTUNE_MAX_CYCLES))
    {
        return 0;
    }

    if ((config->maximum_cycles < config->required_cycles) ||
        (config->maximum_cycles > PID_AUTOTUNE_MAX_CYCLES))
    {
        return 0;
    }

    if ((config->rule != PID_AUTOTUNE_RULE_ZN_PI) &&
        (config->rule != PID_AUTOTUNE_RULE_ZN_PID) &&
        (config->rule != PID_AUTOTUNE_RULE_TYREUS_LUYBEN_PID))
    {
        return 0;
    }

    return 1;
}


static void PID_AutoTune_WriteOutput(PID_AutoTune *tuner,
                                     PID *pid,
                                     float equivalent_temperature)
{
    float limited_output;
    float output_voltage;

    if ((tuner == NULL) || (pid == NULL))
    {
        return;
    }

    limited_output =
        PID_AutoTune_LimitFloat(equivalent_temperature,
                                tuner->config.output_min,
                                tuner->config.output_max);

    tuner->current_output = limited_output;
    pid->Tec_Current_SetPoint = limited_output;

    output_voltage = TEC_TempV(limited_output);
    CTRL_TempVout(tuner->channel, output_voltage);
}


static void PID_AutoTune_RestoreSavedState(PID_AutoTune *tuner,
                                           PID *pid)
{
    if ((tuner == NULL) || (pid == NULL))
    {
        return;
    }

    pid->Tec_Kp = tuner->saved_kp;
    pid->Tec_Ki = tuner->saved_ki;
    pid->Tec_Kd = tuner->saved_kd;

    Formula_TecPid_Reset(pid);

    PID_AutoTune_WriteOutput(tuner,
                             pid,
                             tuner->saved_output);
}


static PID_AutoTuneState PID_AutoTune_Fail(PID_AutoTune *tuner,
                                           PID *pid,
                                           PID_AutoTuneError error)
{
    if (tuner == NULL)
    {
        return PID_AUTOTUNE_STATE_ERROR;
    }

    tuner->error = error;
    tuner->state = PID_AUTOTUNE_STATE_ERROR;
    tuner->progress = 0U;

    PID_AutoTune_RestoreSavedState(tuner, pid);

    return tuner->state;
}


static void PID_AutoTune_UpdateProgress(PID_AutoTune *tuner)
{
    uint32_t progress;

    if (tuner == NULL)
    {
        return;
    }

    if (tuner->state == PID_AUTOTUNE_STATE_FINISHED)
    {
        tuner->progress = 100U;
        return;
    }

    if (!PID_AutoTune_IsRunning(tuner))
    {
        return;
    }

    /*
     * 10% means started, 90% means enough cycles have been collected.
     */
    progress = 10U +
               ((uint32_t)tuner->cycle_count * 80U) /
               (uint32_t)tuner->config.required_cycles;

    if (progress > 90U)
    {
        progress = 90U;
    }

    tuner->progress = (uint8_t)progress;
}


static void PID_AutoTune_EnterHigh(PID_AutoTune *tuner,
                                   PID *pid,
                                   float actual_temperature,
                                   uint32_t now_ms)
{
    tuner->state = PID_AUTOTUNE_STATE_RELAY_HIGH;
    tuner->half_cycle_start_time_ms = now_ms;
    tuner->half_cycle_min = actual_temperature;
    tuner->half_cycle_max = actual_temperature;

    PID_AutoTune_WriteOutput(tuner,
                             pid,
                             tuner->output_high);
}


static void PID_AutoTune_EnterLow(PID_AutoTune *tuner,
                                  PID *pid,
                                  float actual_temperature,
                                  uint32_t now_ms)
{
    tuner->state = PID_AUTOTUNE_STATE_RELAY_LOW;
    tuner->half_cycle_start_time_ms = now_ms;
    tuner->half_cycle_min = actual_temperature;
    tuner->half_cycle_max = actual_temperature;

    PID_AutoTune_WriteOutput(tuner,
                             pid,
                             tuner->output_low);
}


static int PID_AutoTune_CyclesAreStable(const PID_AutoTune *tuner,
                                        uint8_t start_index,
                                        uint8_t count,
                                        float *average_amplitude,
                                        float *average_period_s)
{
    uint8_t index;
    float amplitude_sum;
    float period_sum;
    float amplitude_mean;
    float period_mean;

    if ((tuner == NULL) ||
        (average_amplitude == NULL) ||
        (average_period_s == NULL) ||
        (count == 0U))
    {
        return 0;
    }

    amplitude_sum = 0.0f;
    period_sum = 0.0f;

    for (index = 0U; index < count; index++)
    {
        amplitude_sum +=
            tuner->cycle_amplitude[start_index + index];

        period_sum +=
            tuner->cycle_period_s[start_index + index];
    }

    amplitude_mean = amplitude_sum / (float)count;
    period_mean = period_sum / (float)count;

    if ((amplitude_mean <= PID_AUTOTUNE_EPSILON) ||
        (period_mean <= PID_AUTOTUNE_EPSILON))
    {
        return 0;
    }

    for (index = 0U; index < count; index++)
    {
        float amplitude_deviation;
        float period_deviation;

        amplitude_deviation =
            PID_AutoTune_AbsFloat(
                tuner->cycle_amplitude[start_index + index] -
                amplitude_mean) /
            amplitude_mean;

        period_deviation =
            PID_AutoTune_AbsFloat(
                tuner->cycle_period_s[start_index + index] -
                period_mean) /
            period_mean;

        if ((amplitude_deviation >
             tuner->config.stability_tolerance) ||
            (period_deviation >
             tuner->config.stability_tolerance))
        {
            return 0;
        }
    }

    *average_amplitude = amplitude_mean;
    *average_period_s = period_mean;

    return 1;
}

//Õû¶¨½á¹ûÏÈ±£´æÔÚ×Ô¶¯Õû¶¨¶ÔÏóÖÐ£¬¼ÆËãpid²ÎÊý¸ù¾Ý·ù¶È¡¢ÖÜÆÚµÈ
static int PID_AutoTune_CalculateGains(PID_AutoTune *tuner)
{
    uint8_t start_index;
    uint8_t count;
    float oscillation_amplitude;
    float ultimate_period_s;
    float ultimate_gain;
    float sample_time_s;
    float continuous_kp;
    float integral_time_s;
    float derivative_time_s;

    if (tuner == NULL)
    {
        return 0;
    }

    count = tuner->config.required_cycles;

    if (tuner->cycle_count < count)
    {
        return 0;
    }

    /*
     * Always use the newest required_cycles measurements.
     */
    start_index = tuner->cycle_count - count;

    if (!PID_AutoTune_CyclesAreStable(tuner,
                                      start_index,
                                      count,
                                      &oscillation_amplitude,
                                      &ultimate_period_s))
    {
        return 0;
    }

    if ((oscillation_amplitude * 2.0f) <
        tuner->config.minimum_peak_to_peak)
    {
        tuner->error =
            PID_AUTOTUNE_ERROR_OSCILLATION_TOO_SMALL;
        return -1;
    }

    /*
     * Symmetric relay test:
     *
     *     Ku = 4d / (pi * a)
     *
     * d: relay_step
     * a: process oscillation amplitude
     */
    ultimate_gain =
        (4.0f * tuner->config.relay_step) /
        (PID_AUTOTUNE_PI * oscillation_amplitude);

    sample_time_s =
        (float)tuner->config.sample_time_ms / 1000.0f;

    if ((ultimate_gain <= PID_AUTOTUNE_EPSILON) ||
        (ultimate_period_s <= PID_AUTOTUNE_EPSILON) ||
        (sample_time_s <= PID_AUTOTUNE_EPSILON))
    {
        tuner->error =
            PID_AUTOTUNE_ERROR_OSCILLATION_TOO_SMALL;
        return -1;
    }

    continuous_kp = 0.0f;
    integral_time_s = 0.0f;
    derivative_time_s = 0.0f;

    switch (tuner->config.rule)
    {
        case PID_AUTOTUNE_RULE_ZN_PI:
        {
            continuous_kp = 0.45f * ultimate_gain;
            integral_time_s = ultimate_period_s / 1.2f;
            derivative_time_s = 0.0f;
            break;
        }

        case PID_AUTOTUNE_RULE_ZN_PID:
        {
            continuous_kp = 0.60f * ultimate_gain;
            integral_time_s = 0.50f * ultimate_period_s;
            derivative_time_s = 0.125f * ultimate_period_s;
            break;
        }

        case PID_AUTOTUNE_RULE_TYREUS_LUYBEN_PID:
        default:
        {
            /*
             * More conservative than classic Ziegler-Nichols.
             */
            continuous_kp = ultimate_gain / 2.2f;
            integral_time_s = 2.2f * ultimate_period_s;
            derivative_time_s = ultimate_period_s / 6.3f;
            break;
        }
    }

    /*
     * Convert continuous parallel PID parameters to the exact
     * incremental-discrete coefficients used by formula.c:
     *
     * Kp_discrete = Kp_continuous
     * Ki_discrete = Kp_continuous * Ts / Ti
     * Kd_discrete = Kp_continuous * Td / Ts
     */
    tuner->result_kp = continuous_kp; //´ËÊ±»¹Ö»ÊÇ±£´æÔÚ×Ô¶¯Õû¶¨¶ÔÏóÖÐ

    if (integral_time_s > PID_AUTOTUNE_EPSILON)
    {
        tuner->result_ki =
            continuous_kp * sample_time_s /
            integral_time_s;
    }
    else
    {
        tuner->result_ki = 0.0f;
    }

    if (derivative_time_s > PID_AUTOTUNE_EPSILON)
    {
        tuner->result_kd =
            continuous_kp * derivative_time_s /
            sample_time_s;
    }
    else
    {
        tuner->result_kd = 0.0f;
    }

    if (!PID_AutoTune_IsFiniteFloat(tuner->result_kp) ||
        !PID_AutoTune_IsFiniteFloat(tuner->result_ki) ||
        !PID_AutoTune_IsFiniteFloat(tuner->result_kd) ||
        (tuner->result_kp < 0.0f) ||
        (tuner->result_ki < 0.0f) ||
        (tuner->result_kd < 0.0f))
    {
        tuner->error =
            PID_AUTOTUNE_ERROR_OSCILLATION_TOO_SMALL;
        return -1;
    }

    tuner->ultimate_gain = ultimate_gain;
    tuner->ultimate_period_s = ultimate_period_s;

    return 1;
}


static int PID_AutoTune_StoreCycle(PID_AutoTune *tuner,
                                   float amplitude,
                                   float period_s)
{
    uint8_t index;

    if (tuner == NULL)
    {
        return 0;
    }

    if (!PID_AutoTune_IsFiniteFloat(amplitude) ||
        !PID_AutoTune_IsFiniteFloat(period_s) ||
        (amplitude <= 0.0f) ||
        (period_s <= 0.0f))
    {
        return 0;
    }

    if (tuner->cycle_count < PID_AUTOTUNE_MAX_CYCLES)
    {
        index = tuner->cycle_count;
        tuner->cycle_count++;
    }
    else
    {
        /*
         * Keep the newest measurements if the fixed array is full.
         */
        for (index = 1U;
             index < PID_AUTOTUNE_MAX_CYCLES;
             index++)
        {
            tuner->cycle_amplitude[index - 1U] =
                tuner->cycle_amplitude[index];

            tuner->cycle_period_s[index - 1U] =
                tuner->cycle_period_s[index];
        }

        index = PID_AUTOTUNE_MAX_CYCLES - 1U;
    }

    tuner->cycle_amplitude[index] = amplitude;
    tuner->cycle_period_s[index] = period_s;

    PID_AutoTune_UpdateProgress(tuner);

    return 1;
}

//×Ô¶¯Õû¶¨µÄÄ¬ÈÏÅäÖÃ
void PID_AutoTune_DefaultConfig(PID_AutoTuneConfig *config,
                                float target_temperature,
                                uint32_t sample_time_ms)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    
		//ÖÆÔìÕñµ´µÄ¸ßÊä³öµÍÊä³öãÐÖµ
    config->target_temperature = target_temperature;
    config->relay_step = 2.0f;  
    config->hysteresis = 0.20f;

    config->output_min = 0.0f;
    config->output_max = 60.0f;

    config->safety_temperature_min = 0.0f;
    config->safety_temperature_max = 60.0f;

    config->sample_time_ms = sample_time_ms;

    /*
     * Thermal systems are slow. These defaults allow up to 30 minutes
     * total and 5 minutes for one half-cycle.
     */
    config->total_timeout_ms = 30U * 60U * 1000U;
    config->half_cycle_timeout_ms = 5U * 60U * 1000U;

    config->minimum_peak_to_peak = 0.50f;
    config->stability_tolerance = 0.35f;

    config->required_cycles = 4U; //ÖÁÉÙÒª²É¼¯µ½4¸öÍêÕû¡¢Ïà¶ÔÎÈ¶¨µÄÎÂ¶ÈÕñµ´ÖÜÆÚ£¬²Å»á³¢ÊÔ¼ÆËãpid²ÎÊý
    config->maximum_cycles = 8U;

    config->rule =
        PID_AUTOTUNE_RULE_TYREUS_LUYBEN_PID;

    config->apply_result_automatically = 1U;
}


int PID_AutoTune_Init(PID_AutoTune *tuner,
                      const PID_AutoTuneConfig *config)
{
    if (tuner == NULL)
    {
        return 0;
    }

    memset(tuner, 0, sizeof(*tuner));

    if (!PID_AutoTune_ConfigIsValid(config))
    {
        tuner->state = PID_AUTOTUNE_STATE_ERROR;
        tuner->error = PID_AUTOTUNE_ERROR_BAD_CONFIG;
        return 0;
    }

    tuner->config = *config;
    tuner->state = PID_AUTOTUNE_STATE_IDLE;
    tuner->error = PID_AUTOTUNE_ERROR_NONE;

    return 1;
}

//Æô¶¯×Ô¶¯Õû¶¨Ê±
int PID_AutoTune_Start(PID_AutoTune *tuner,
                       PID *pid,
                       float actual_temperature,
                       uint32_t now_ms,
                       Tec_Channel channel)
{
    float high_output;
    float low_output;

    if ((tuner == NULL) || (pid == NULL))
    {
        if (tuner != NULL)
        {
            tuner->state = PID_AUTOTUNE_STATE_ERROR;
            tuner->error = PID_AUTOTUNE_ERROR_BAD_ARGUMENT;
        }

        return 0;
    }

    if (!PID_AutoTune_ConfigIsValid(&tuner->config))
    {
        tuner->state = PID_AUTOTUNE_STATE_ERROR;
        tuner->error = PID_AUTOTUNE_ERROR_BAD_CONFIG;
        return 0;
    }

    if (!PID_AutoTune_IsFiniteFloat(actual_temperature))
    {
        tuner->state = PID_AUTOTUNE_STATE_ERROR;
        tuner->error =
            PID_AUTOTUNE_ERROR_INVALID_TEMPERATURE;
        return 0;
    }

    if ((actual_temperature <=
         tuner->config.safety_temperature_min) ||
        (actual_temperature >=
         tuner->config.safety_temperature_max))
    {
        tuner->state = PID_AUTOTUNE_STATE_ERROR;
        tuner->error = PID_AUTOTUNE_ERROR_SAFETY_LIMIT;
        return 0;
    }

    high_output =
        PID_AutoTune_LimitFloat(
            tuner->config.target_temperature +
            tuner->config.relay_step,
            tuner->config.output_min,
            tuner->config.output_max);

    low_output =
        PID_AutoTune_LimitFloat(
            tuner->config.target_temperature -
            tuner->config.relay_step,
            tuner->config.output_min,
            tuner->config.output_max);

    if ((high_output - low_output) <=
        PID_AUTOTUNE_EPSILON)
    {
        tuner->state = PID_AUTOTUNE_STATE_ERROR;
        tuner->error = PID_AUTOTUNE_ERROR_BAD_CONFIG;
        return 0;
    }

    tuner->saved_kp = pid->Tec_Kp;
    tuner->saved_ki = pid->Tec_Ki;
    tuner->saved_kd = pid->Tec_Kd;
    tuner->saved_output = pid->Tec_Current_SetPoint;

    tuner->channel = channel;

    tuner->start_time_ms = now_ms;
    tuner->last_sample_time_ms =
        now_ms - tuner->config.sample_time_ms;

    tuner->half_cycle_start_time_ms = now_ms;
    tuner->last_upper_cross_time_ms = 0U;

    tuner->upper_cross_valid = 0U;
    tuner->pending_high_peak_valid = 0U;
    tuner->cycle_count = 0U;
    tuner->progress = 10U;

    tuner->output_center =
        tuner->config.target_temperature;
    tuner->output_high = high_output;
    tuner->output_low = low_output;
    tuner->current_output =
        tuner->config.target_temperature;

    tuner->half_cycle_min = actual_temperature;
    tuner->half_cycle_max = actual_temperature;
    tuner->pending_high_peak = actual_temperature;

    memset(tuner->cycle_amplitude,
           0,
           sizeof(tuner->cycle_amplitude));

    memset(tuner->cycle_period_s,
           0,
           sizeof(tuner->cycle_period_s));

    tuner->ultimate_gain = 0.0f;
    tuner->ultimate_period_s = 0.0f;
    tuner->result_kp = 0.0f;
    tuner->result_ki = 0.0f;
    tuner->result_kd = 0.0f;

    tuner->error = PID_AUTOTUNE_ERROR_NONE;

    Formula_TecPid_Reset(pid);

    if (actual_temperature <=
        tuner->config.target_temperature)
    {
        PID_AutoTune_EnterHigh(tuner,
                               pid,
                               actual_temperature,
                               now_ms);
    }
    else
    {
        PID_AutoTune_EnterLow(tuner,
                              pid,
                              actual_temperature,
                              now_ms);
    }

    return 1;
}

//×Ô¶¯Õû¶¨ÔËÐÐÊ±
PID_AutoTuneState PID_AutoTune_Task(PID_AutoTune *tuner,
                                    PID *pid,
                                    float actual_temperature,
                                    uint32_t now_ms)
{
    float upper_threshold;
    float lower_threshold;

    if ((tuner == NULL) || (pid == NULL))
    {
        if (tuner != NULL)
        {
            tuner->state = PID_AUTOTUNE_STATE_ERROR;
            tuner->error = PID_AUTOTUNE_ERROR_BAD_ARGUMENT;
        }

        return PID_AUTOTUNE_STATE_ERROR;
    }

    if (!PID_AutoTune_IsRunning(tuner))
    {
        return tuner->state;
    }

    if (PID_AutoTune_ElapsedMs(
            now_ms,
            tuner->last_sample_time_ms) <
        tuner->config.sample_time_ms)
    {
        return tuner->state;
    }

    tuner->last_sample_time_ms = now_ms;

    if (!PID_AutoTune_IsFiniteFloat(actual_temperature))
    {
        return PID_AutoTune_Fail(
            tuner,
            pid,
            PID_AUTOTUNE_ERROR_INVALID_TEMPERATURE);
    }

    if ((actual_temperature <=
         tuner->config.safety_temperature_min) ||
        (actual_temperature >=
         tuner->config.safety_temperature_max))
    {
        return PID_AutoTune_Fail(
            tuner,
            pid,
            PID_AUTOTUNE_ERROR_SAFETY_LIMIT);
    }

    if (PID_AutoTune_ElapsedMs(
            now_ms,
            tuner->start_time_ms) >
        tuner->config.total_timeout_ms)
    {
        return PID_AutoTune_Fail(
            tuner,
            pid,
            PID_AUTOTUNE_ERROR_TOTAL_TIMEOUT);
    }

    if (PID_AutoTune_ElapsedMs(
            now_ms,
            tuner->half_cycle_start_time_ms) >
        tuner->config.half_cycle_timeout_ms)
    {
        return PID_AutoTune_Fail(
            tuner,
            pid,
            PID_AUTOTUNE_ERROR_HALF_CYCLE_TIMEOUT);
    }

    upper_threshold =
        tuner->config.target_temperature +
        tuner->config.hysteresis;

    lower_threshold =
        tuner->config.target_temperature -
        tuner->config.hysteresis;

    if (actual_temperature < tuner->half_cycle_min)
    {
        tuner->half_cycle_min = actual_temperature;
    }

    if (actual_temperature > tuner->half_cycle_max)
    {
        tuner->half_cycle_max = actual_temperature;
    }

    switch (tuner->state)
    {
        case PID_AUTOTUNE_STATE_RELAY_HIGH:
        {
            /*
             * High output drives temperature upward.
             * The minimum reached during this phase is the low peak.
             */
            if (actual_temperature >= upper_threshold)
            {
                if ((tuner->upper_cross_valid != 0U) &&
                    (tuner->pending_high_peak_valid != 0U))
                {
                    float amplitude;
                    float period_s;
                    int calculate_status;

                    amplitude =
                        (tuner->pending_high_peak -
                         tuner->half_cycle_min) * 0.5f;

                    period_s =
                        (float)PID_AutoTune_ElapsedMs(
                            now_ms,
                            tuner->last_upper_cross_time_ms) /
                        1000.0f;

                    if (!PID_AutoTune_StoreCycle(
                            tuner,
                            amplitude,
                            period_s))
                    {
                        return PID_AutoTune_Fail(
                            tuner,
                            pid,
                            PID_AUTOTUNE_ERROR_OSCILLATION_TOO_SMALL);
                    }

                    if (tuner->cycle_count >=
                        tuner->config.required_cycles)
                    {
                        tuner->state =
                            PID_AUTOTUNE_STATE_CALCULATING;

                        calculate_status =
                            PID_AutoTune_CalculateGains(tuner);

                        if (calculate_status > 0)
                        {
                            if (tuner->config.
                                apply_result_automatically != 0U)
                            {
                                (void)PID_AutoTune_ApplyResult(
                                    tuner,
                                    pid);
                            }

                            PID_AutoTune_WriteOutput(
                                tuner,
                                pid,
                                tuner->config.target_temperature);

                            tuner->state =
                                PID_AUTOTUNE_STATE_FINISHED;
                            tuner->error =
                                PID_AUTOTUNE_ERROR_NONE;
                            tuner->progress = 100U;

                            return tuner->state;
                        }

                        if (calculate_status < 0)
                        {
                            return PID_AutoTune_Fail(
                                tuner,
                                pid,
                                tuner->error);
                        }

                        if (tuner->cycle_count >=
                            tuner->config.maximum_cycles)
                        {
                            return PID_AutoTune_Fail(
                                tuner,
                                pid,
                                PID_AUTOTUNE_ERROR_OSCILLATION_UNSTABLE);
                        }
                    }
                }

                tuner->last_upper_cross_time_ms = now_ms;
                tuner->upper_cross_valid = 1U;
                tuner->pending_high_peak_valid = 0U;

                PID_AutoTune_EnterLow(tuner,
                                      pid,
                                      actual_temperature,
                                      now_ms);
            }

            break;
        }

        case PID_AUTOTUNE_STATE_RELAY_LOW:
        {
            /*
             * Low output drives temperature downward.
             * Thermal inertia may still push temperature upward first;
             * therefore the maximum over the whole low-output half-cycle
             * is used as the high peak.
             */
            if (actual_temperature <= lower_threshold)
            {
                tuner->pending_high_peak =
                    tuner->half_cycle_max;

                tuner->pending_high_peak_valid = 1U;

                PID_AutoTune_EnterHigh(tuner,
                                       pid,
                                       actual_temperature,
                                       now_ms);
            }

            break;
        }

        case PID_AUTOTUNE_STATE_CALCULATING:
        {
            /*
             * Calculation is performed immediately at a crossing, so
             * normal execution should not remain in this state.
             */
            return PID_AutoTune_Fail(
                tuner,
                pid,
                PID_AUTOTUNE_ERROR_BAD_ARGUMENT);
        }

        default:
        {
            break;
        }
    }

    PID_AutoTune_UpdateProgress(tuner);

    return tuner->state;
}


void PID_AutoTune_Abort(PID_AutoTune *tuner,
                        PID *pid)
{
    if ((tuner == NULL) || (pid == NULL))
    {
        return;
    }

    if (PID_AutoTune_IsRunning(tuner))
    {
        PID_AutoTune_RestoreSavedState(tuner, pid);
    }

    tuner->state = PID_AUTOTUNE_STATE_ABORTED;
    tuner->error = PID_AUTOTUNE_ERROR_NONE;
    tuner->progress = 0U;
}

//ÕæÕý´«¸øÔöÁ¿Ê½pidµÄÎ»ÖÃ
int PID_AutoTune_ApplyResult(const PID_AutoTune *tuner,
                             PID *pid)
{
    if ((tuner == NULL) || (pid == NULL))
    {
        return 0;
    }

    if (!PID_AutoTune_HasResult(tuner))
    {
        return 0;
    }

    pid->Tec_Kp = tuner->result_kp;
    pid->Tec_Ki = tuner->result_ki;
    pid->Tec_Kd = tuner->result_kd;

    Formula_TecPid_Reset(pid);

    return 1;
}


int PID_AutoTune_IsRunning(const PID_AutoTune *tuner)
{
    if (tuner == NULL)
    {
        return 0;
    }

    return ((tuner->state ==
             PID_AUTOTUNE_STATE_RELAY_HIGH) ||
            (tuner->state ==
             PID_AUTOTUNE_STATE_RELAY_LOW) ||
            (tuner->state ==
             PID_AUTOTUNE_STATE_CALCULATING));
}


int PID_AutoTune_HasResult(const PID_AutoTune *tuner)
{
    if (tuner == NULL)
    {
        return 0;
    }

    if ((tuner->state != PID_AUTOTUNE_STATE_FINISHED) &&
        (tuner->ultimate_gain <= 0.0f))
    {
        return 0;
    }

    return (PID_AutoTune_IsFiniteFloat(tuner->result_kp) &&
            PID_AutoTune_IsFiniteFloat(tuner->result_ki) &&
            PID_AutoTune_IsFiniteFloat(tuner->result_kd) &&
            (tuner->result_kp >= 0.0f) &&
            (tuner->result_ki >= 0.0f) &&
            (tuner->result_kd >= 0.0f));
}


uint8_t PID_AutoTune_GetProgress(const PID_AutoTune *tuner)
{
    if (tuner == NULL)
    {
        return 0U;
    }

    return tuner->progress;
}


int PID_Control_Init(PID_Control *control,
                     PID *pid,
                     float target_temperature,
                     uint32_t control_period_ms,
                     Tec_Channel channel)
{
    if ((control == NULL) || (pid == NULL))
    {
        return 0;
    }

    memset(control, 0, sizeof(*control));

    control->pid = pid;
    control->channel = channel;
    control->target_temperature = target_temperature;
    control->control_period_ms = control_period_ms;
    control->mode = PID_CONTROL_MODE_MANUAL;

    PID_AutoTune_DefaultConfig(&control->config,
                               target_temperature,
                               control_period_ms);

    return PID_AutoTune_Init(&control->autotune,
                             &control->config);
}


void PID_Control_RequestAutoTune(PID_Control *control)
{
    if (control != NULL)
    {
        control->autotune_request = 1U;
    }
}


void PID_Control_SetManual(PID_Control *control)
{
    if ((control == NULL) || (control->pid == NULL))
    {
        return;
    }

    if (PID_AutoTune_IsRunning(&control->autotune))
    {
        PID_AutoTune_Abort(&control->autotune,
                           control->pid);
    }

    Formula_TecPid_Reset(control->pid);
    control->autotune_request = 0U;
    control->mode = PID_CONTROL_MODE_MANUAL;
}


void PID_Control_EnablePID(PID_Control *control,
                           uint32_t now_ms)
{
    if ((control == NULL) || (control->pid == NULL))
    {
        return;
    }

    if (PID_AutoTune_IsRunning(&control->autotune))
    {
        PID_AutoTune_Abort(&control->autotune,
                           control->pid);
    }

    Formula_TecPid_Reset(control->pid);
    control->last_pid_time_ms = now_ms;
    control->autotune_request = 0U;
    control->mode = PID_CONTROL_MODE_CLOSED_LOOP;
}


PID_ControlMode PID_Control_Task(PID_Control *control,
                                 float actual_temperature,
                                 uint8_t temperature_valid,
                                 uint32_t now_ms)
{
    PID_AutoTuneState state;

    if ((control == NULL) || (control->pid == NULL))
    {
        return PID_CONTROL_MODE_MANUAL;
    }

    if (temperature_valid == 0U)
    {
        uint8_t pending_request = control->autotune_request;

        PID_Control_SetManual(control);

        /* ??????????????? */
        control->autotune_request = pending_request;
        return control->mode;
    }

    if (control->autotune_request != 0U)
    {
        control->autotune_request = 0U;

        if (PID_AutoTune_Start(&control->autotune,
                               control->pid,
                               actual_temperature,
                               now_ms,
                               control->channel))
        {
            control->mode = PID_CONTROL_MODE_AUTOTUNE;
        }
        else
        {
            control->mode = PID_CONTROL_MODE_MANUAL;
        }
    }

    switch (control->mode)
    {
        case PID_CONTROL_MODE_AUTOTUNE:
        {
            state = PID_AutoTune_Task(&control->autotune,
                                      control->pid,
                                      actual_temperature,
                                      now_ms);

            if (state == PID_AUTOTUNE_STATE_FINISHED)
            {
                /* ??:???????????PID */
                Formula_TecPid_Reset(control->pid);
                control->last_pid_time_ms = now_ms;
                control->mode = PID_CONTROL_MODE_CLOSED_LOOP;
            }
            else if ((state == PID_AUTOTUNE_STATE_ERROR) ||
                     (state == PID_AUTOTUNE_STATE_ABORTED))
            {
                /* ??:??????????? */
                PID_Control_SetManual(control);
            }

            break;
        }

        case PID_CONTROL_MODE_CLOSED_LOOP:
        {
            if (PID_AutoTune_ElapsedMs(now_ms,
                                      control->last_pid_time_ms) >=
                control->control_period_ms)
            {
                control->last_pid_time_ms = now_ms;

                Formula_TecTemperature_PID(
                    control->target_temperature,
                    actual_temperature,
                    control->pid,
                    control->channel);
            }

            break;
        }

        case PID_CONTROL_MODE_MANUAL:
        default:
        {
            break;
        }
    }

    return control->mode;
}


PID_ControlMode PID_Control_GetMode(const PID_Control *control)
{
    if (control == NULL)
    {
        return PID_CONTROL_MODE_MANUAL;
    }

    return control->mode;
}

```
# 在 main.c 中调用

在调用pid 函数时，应显式包含：

#include "pid.h"

建议放在：

/* USER CODE BEGIN Includes */  
#include "pid.h"  
​  
/* USER CODE END Includes */

全局对象和模式定义：

/* USER CODE BEGIN PD */
#define TEC1_TARGET_TEMPERATURE  25.0f  
#define TEC1_CONTROL_PERIOD_MS   200U  

/* USER CODE END PD */


/* USER CODE BEGIN PV */

static PID TEC1_PID;
PID_Control TEC1_Control；

  /* USER CODE BEGIN PV */


---

## 初始化代码

在外设初始化、DAC 、增量式PID初始化完成后：


if (!PID_AutoTune_Init(  
        &TEC1_AutoTune,  
        &TEC1_AutoTuneConfig))  
{  
    Error_Handler();  
}  

/*  
 * 上电自动整定一次。  
 * 这里只提交请求，必须等到获得有效温度后才能真正启动。  
 */  
 PID_Control_RequestAutoTune(&TEC1_Control);

如果不希望上电自动整定，就不要设置请求，改为收到串口命令后执行：

PID_Control_RequestAutoTune(&TEC1_Control);

---

## 主循环调用流程
将PID控制任务的位置放在
```
ntc_data_valid =
    NTC_NodeVoltageToTemperature()后面
```

```
    dac_temp_valid =
    NTC_NodeVoltageToTemperature()前面
```
PID控制任务：
```
	PID_Control_Task(&TEC1_Control,
                     (float)ntc_temperature,
                     ntc_data_valid,
                     HAL_GetTick();
```




## 一句话总结

自动整定 PID：  
用高低档输出让真实温度产生受控振荡，  
根据振荡幅值和周期计算得到合适的 Kp、Ki、Kd。  
​  
增量式 PID：  
使用自动整定得到的 Kp、Ki、Kd，  
每个控制周期根据目标温度和实际温度的误差，  
计算输出增量并持续修正 DAC0，  
最终让温度稳定在目标值附近。