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