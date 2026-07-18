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



/* PID 初始化 */
void Formula_TecPid_Init(PID *pid,
                         float kp,
                         float ki,
                         float kd,
                         float init_setpoint);

/*
 * PID 状态清零
 *
 */
void Formula_TecPid_Reset(PID *pid);


void Formula_TecTemperature_PID(float setValue,
                                float actualValue,
                                PID *pid,
                                Tec_Channel tec_channel);

											
															
/* 温度反算成 NTC 电压 */
float TEC_TempV(float Tec_Temp);
										
//输出TEC控制电压
void CTRL_TempVout(Tec_Channel tec_channel, float voltage);															 
															 
#ifdef __cplusplus
}
#endif

#endif


