#ifndef __THERMAL_H
#define __THERMAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * NTC电路:
 *
 *                         2.5V
 *                           |
 *               +-----------+-----------+
 *               |                       |
 *             10kO                    10kO
 *               |                       |
 *              N1                      N2
 *               |                       |
 *          10kO NTC                  10kO
 *               |                       |
 *              GND                     GND
 *
 * ADS1256差分输入:
 *
 *     Vdiff = V(N1) - V(N2)
 *
 * ?NTC为10kO,温度约为25??:
 *
 *     V(N1) = 1.25V
 *     V(N2) = 1.25V
 *     Vdiff = 0V
 */


/* 供电电压 */
#define NTC_BRIDGE_VOLTAGE_V       (2.5)

/* ??NTC???????? */
#define NTC_FIXED_OHM              (10000.0)

/* ???????? */
#define NTC_REF_TOP_OHM            (10000.0)
#define NTC_REF_BOTTOM_OHM         (10000.0)

/* NTC???? */
#define NTC_R25_OHM                (10000.0)

/*
 * NTC?B值
 * ??????NTC??????
 *
 * ???:
 * 3435
 * 3950
 * 3977
 * 4250
 */
#define NTC_BETA_VALUE             (3950.0)

/* NTC???? */
#define NTC_NOMINAL_TEMP_C         (25.0)

/* ?????????? */
#define NTC_INVALID_TEMP_C         (-273.15)


/**
 * @brief 差分电压转换为NTC温度和电阻
 *
 * @param diff_voltage
 *        ADS1256采集到的差分电压:
 *        V(N1) - V(N2)
 *
 * @param temperature_c
 *        ????,???
 *
 * @param resistance_ohm
 *        ??NTC电阻,??O
 *
 * @return
 *        1:????
 *        0:???????????????
 */
int NTC_DiffVoltageToTemperature(double diff_voltage,
                                 double *temperature_c,
                                 double *resistance_ohm);


/**
 * @brief ???????????
 *
 * ????????????
 *
 * @param temperature_c
 *        ??,???
 *
 * @param diff_voltage
 *        ????????,??V
 *
 * @return
 *        1:????
 *        0:????
 */
int NTC_TemperatureToDiffVoltage(double temperature_c,
                                 double *diff_voltage);


//ntc分压转为温度
int NTC_NodeVoltageToTemperature(
    double node_voltage_v,
    double *temperature_c,
    double *resistance_ohm
);


/*
 * ???thermal.c??????
 *
 * ??:
 * voltage????????V(N1)-V(N2),
 * ????0~5V???????
 */
double NTC_VoltageToTemp(double diff_voltage);
double NTC_TempToVoltage(double temperature_c);
		


#ifdef __cplusplus
}
#endif

#endif

