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


//NTC参数
#define FORMULA_NTC_VCC              2.5f
#define FORMULA_NTC_FIXED_R          10000.0f
#define FORMULA_NTC_R25              10000.0f
#define FORMULA_NTC_BETA             3950.0f
#define FORMULA_NTC_T25_K            298.15f
/*
 * PID 输出限幅
 *
 */
#define FORMULA_TEC_PID_OUT_MIN      0.0f
#define FORMULA_TEC_PID_OUT_MAX      300.0f


/*
 * 误差分段阈值
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
 * 小误差平滑系数，误差融合
 *
 *
 * if(fabs(Error) < 0.05)
 * {
 *     Error = Error * 0.7 + LastError * 0.3;
 * }
 */
#define FORMULA_TEC_ERR_FILTER_NOW   0.7f
#define FORMULA_TEC_ERR_FILTER_LAST  0.3f

/*
 * 增量比例系数
 *
 *
 * pid->PIDInc = (P_Term + I_Term + D_Term) * 0.5f;
 * pid->PIDInc = (P_Term * 0.5f + I_Term + D_Term) * 0.5f;
 * }
 */
#define FORMULA_TEC_INC   0.5f



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
 * PID 初始化
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
 * PID 状态清零
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
 * 温度 -> NTC 节点电压
 *
 * ??:
 *      PID ???????????? Tec_Current_SetPoint?
 *      ?????????? NTC ??????
 *
 * 公式:
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
 * TEC 控制电压输出
 *
 * ??? TEC ????? DAC8568 ???
 *
 * ????:
 *      TEC_1 -> DAC8568 OutA
 *      TEC_2 -> DAC8568 OutC
 *
 * 输出控制电压
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
 * 设置PID 参数
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
 * PID 状态复位
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
 * TEC温度计算
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
     * 1. 计算当前误差
     *
     * error > 0:
     *      目标比实际高，需要升温
     *
     * error < 0:
     *      目标比实际低，降温
     */
    Error = setValue - actualValue;

    /*
     * 2. 小误差区域平滑
     *
     *误差很小时，直接使用当前误差可能会导致输出抖动
		 *所以这里把当前误差和上一次误差做加权平均可以
     */
if (fabsf(Error) < FORMULA_TEC_ERR_BIG)
    {
        Error = Error * FORMULA_TEC_ERR_FILTER_NOW
              + pid->LastError * FORMULA_TEC_ERR_FILTER_LAST;
    }


    /*
     * 3. PID计算
     *
     */
    P_Term = pid->Tec_Kp * (Error - pid->LastError);//Kp * (本次误差-上次误差)，误差变化快，p就大


    I_Term = pid->Tec_Ki * Error;  //跟当前误差有关，有误差就往输出里加一点点，消除稳态误差

     D_Term = pid->Tec_Kd *
             (Error - 2.0f * pid->LastError + pid->PrevError);  //误差变化太大，D会抑制输出，减少过冲

    /*
     * 4. 根据误差大小调整pid增量
     *
     * 大:
     *      正常输出PID增量
     *
     * 中等:
     *      PID增量减半
     *
     * 小:
     *      P项减半，总输出再减半
     */
    if (fabsf(Error) > FORMULA_TEC_ERR_BIG)
    {
        pid->PIDInc = P_Term + I_Term + D_Term;
    }
    else if (fabsf(Error) > FORMULA_TEC_ERR_SMALL)
    {
        pid->PIDInc = (P_Term + I_Term + D_Term) * FORMULA_TEC_INC;
    }
    else
    {
        pid->PIDInc = (P_Term * FORMULA_TEC_INC + I_Term + D_Term) * FORMULA_TEC_INC;
    }

    /*
     * 5. 累加PID增量
     *
     * ??:
     * ???????????,?????
     */
    pid->Tec_Current_SetPoint += pid->PIDInc; 

    /*
     * 6. 更新历史误差
     */
    pid->PrevError = pid->LastError;
    pid->LastError = Error;

    /*
     * 7. 输出限幅
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
    output_voltage = TEC_TempV(pid->Tec_Current_SetPoint); //温度通过B值公式转换为电压

    CTRL_TempVout(tec_channel, output_voltage); //得到电压后DAC输出该控制电压

}


