#include "thermal.h"

#include <math.h>
#include <stddef.h>


#define NTC_KELVIN_OFFSET          (273.15)
#define NTC_VOLTAGE_EPSILON        (0.000001)


/**
 * @brief 计算右侧参考支路VN2电压
 *
 * ??:
 *
 * 2.5V -- Rtop -- N2 -- Rbottom -- GND
 */
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


/**
 * @brief 差分电压转换为NTC温度和阻值
 */
int NTC_DiffVoltageToTemperature(double diff_voltage,
                                 double *temperature_c,
                                 double *resistance_ohm)
{
    double node_voltage_v;

    node_voltage_v =
        diff_voltage +
        NTC_GetReferenceNodeVoltage();

    return NTC_NodeVoltageToTemperature(
        node_voltage_v,
        temperature_c,
        resistance_ohm
    );
}


/**
 * @brief 得到温度转换为差分电压
 */
int NTC_TemperatureToDiffVoltage(double temperature_c,
                                 double *diff_voltage)
{
    double temperature_k;
    double nominal_temperature_k;

    double ntc_resistance;
    double ntc_node_voltage;
    double reference_node_voltage;


    if (diff_voltage == NULL)
    {
        return 0;
    }


    if ((NTC_BRIDGE_VOLTAGE_V <= 0.0f) ||
        (NTC_FIXED_OHM <= 0.0f) ||
        (NTC_REF_TOP_OHM <= 0.0f) ||
        (NTC_REF_BOTTOM_OHM <= 0.0f) ||
        (NTC_R25_OHM <= 0.0f) ||
        (NTC_BETA_VALUE <= 0.0f))
    {
        return 0;
    }


    temperature_k =
        temperature_c +
        NTC_KELVIN_OFFSET;


    nominal_temperature_k =
        NTC_NOMINAL_TEMP_C +
        NTC_KELVIN_OFFSET;


    if ((temperature_k <= 0.0f) ||
        (nominal_temperature_k <= 0.0f))
    {
        return 0;
    }


    /*
     * 根据温度计算NTC电阻:
     *
     * R(T) =
     * R25 *
     * exp[B * (1/T - 1/T25)]
     */
    ntc_resistance =
        NTC_R25_OHM *
        exp(
            NTC_BETA_VALUE *
            (
                (1.0f / temperature_k) -
                (1.0f / nominal_temperature_k)
            )
        );


    /*
     * ??N1参考电压
     */
    ntc_node_voltage =
        NTC_BRIDGE_VOLTAGE_V *
        ntc_resistance /
        (
            NTC_FIXED_OHM +
            ntc_resistance
        );


    /*
     * ??N2?????
     */
    reference_node_voltage =
        NTC_GetReferenceNodeVoltage();


    /*
     * ADS1256??????:
     *
     * Vdiff = V(N1) - V(N2)
     */
    *diff_voltage =
        ntc_node_voltage -
        reference_node_voltage;


    return 1;
}


/**
 * @brief ?????:???????
 */
double NTC_VoltageToTemp(double voltage)
{
    double temperature_c;
    double resistance_ohm;


    if (NTC_DiffVoltageToTemperature(
            voltage,
            &temperature_c,
            &resistance_ohm) == 0)
    {
        return NTC_INVALID_TEMP_C;
    }


    return temperature_c;
}


/**
 * @brief 温度转差分电压
 */
double NTC_TempToVoltage(double temp)
{
    double diff_voltage;


    if (NTC_TemperatureToDiffVoltage(
            temp,
            &diff_voltage) == 0)
    {
        return 0.0f;
    }


    return diff_voltage;
}

//节点电压转温度函数
int NTC_NodeVoltageToTemperature(
    double node_voltage_v,
    double *temperature_c,
    double *resistance_ohm)
{
    double ntc_resistance;
    double nominal_temperature_k;
    double inverse_temperature;
    double temperature_k;

    if ((temperature_c == NULL) ||
        (resistance_ohm == NULL))
    {
        return 0;
    }

    if ((node_voltage_v <= NTC_VOLTAGE_EPSILON) ||
        (node_voltage_v >=
         (NTC_BRIDGE_VOLTAGE_V -
          NTC_VOLTAGE_EPSILON)))
    {
        return 0;
    }

    ntc_resistance =
        NTC_FIXED_OHM *
        node_voltage_v /
        (NTC_BRIDGE_VOLTAGE_V -
         node_voltage_v);

    if (ntc_resistance <= 0.0)
    {
        return 0;
    }

		//电阻转为温度
    nominal_temperature_k =
        NTC_NOMINAL_TEMP_C +
        NTC_KELVIN_OFFSET;

    inverse_temperature =
        (1.0 / nominal_temperature_k) +
        log(ntc_resistance / NTC_R25_OHM) /
        NTC_BETA_VALUE;

    if (inverse_temperature <= 0.0)
    {
        return 0;
    }

    temperature_k =
        1.0 / inverse_temperature;

    *temperature_c =
        temperature_k -
        NTC_KELVIN_OFFSET;

    *resistance_ohm =
        ntc_resistance;

    return 1;
}

