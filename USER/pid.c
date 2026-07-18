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

//整定结果先保存在自动整定对象中，计算pid参数根据幅度、周期等
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
    tuner->result_kp = continuous_kp; //此时还只是保存在自动整定对象中

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

//自动整定的默认配置
void PID_AutoTune_DefaultConfig(PID_AutoTuneConfig *config,
                                float target_temperature,
                                uint32_t sample_time_ms)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    
		//制造振荡的高输出低输出阈值
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

    config->required_cycles = 4U; //至少要采集到4个完整、相对稳定的温度振荡周期，才会尝试计算pid参数
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

//启动自动整定时
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

//自动整定运行时
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

//真正传给增量式pid的位置
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
