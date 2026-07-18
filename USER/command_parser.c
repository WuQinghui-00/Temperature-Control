#include "command_parser.h"
#include "bsp_DAC8568.h"
#include "pid.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* 根据实际硬件能力修改允许范围 */
#define TEMPERATURE_MIN_mC    (-40000L)   /* -40.000 ℃ */
#define TEMPERATURE_MAX_mC    (150000L)   /* 150.000 ℃ */

#define CURRENT_MIN_uA        (0L)        /* 0.000 mA */
#define CURRENT_MAX_uA        (500000L)   /* 500.000 mA */

#define DAC_MIN_mV            (0L)
#define DAC_MAX_mV            (5000L)


/* 供其他控制模块使用的目标值 */
volatile int32_t g_target_temperature_mC = 25000;
volatile int32_t g_target_current_uA     = 100000;

extern PID_Control TEC1_Control;

/**
 * @brief 判断字符是否可以出现在指令名称中。
 *
 * 支持大写字母、数字和下划线。
 * 指令首字符必须是大写字母。
 */
static bool Command_IsValidNameChar(char ch)
{
    if ((ch >= 'A') && (ch <= 'Z'))
    {
        return true;
    }

    if ((ch >= '0') && (ch <= '9'))
    {
        return true;
    }

    if (ch == '_')
    {
        return true;
    }

    return false;
}

/**
 * @brief 将字符串转换为 int32_t。
 *
 * 支持：
 * 25001
 * -40000
 * +1000
 */
static CommandParseStatus_t Command_ParseInt32(const char *text,
                                               int32_t *value)
{
    int64_t result = 0;
    int sign = 1;

    if ((text == NULL) || (value == NULL) || (*text == '\0'))
    {
        return CMD_PARSE_BAD_PARAMETER;
    }

    if (*text == '-')
    {
        sign = -1;
        text++;
    }
    else if (*text == '+')
    {
        text++;
    }

    if (*text == '\0')
    {
        return CMD_PARSE_BAD_PARAMETER;
    }

    while (*text != '\0')
    {
        if ((*text < '0') || (*text > '9'))
        {
            return CMD_PARSE_BAD_PARAMETER;
        }

        result = result * 10 + (*text - '0');

        if (result > 2147483648LL)
        {
            return CMD_PARSE_PARAMETER_OVERFLOW;
        }

        text++;
    }

    result *= sign;

    if ((result > 2147483647LL) || (result < -2147483648LL))
    {
        return CMD_PARSE_PARAMETER_OVERFLOW;
    }

    *value = (int32_t)result;

    return CMD_PARSE_OK;
}

/**
 * @brief 将千分之一单位整数格式化为 xxx.xxx。
 *
 * 例如：
 * 25001  -> "25.001"
 * -40000 -> "-40.000"
 */
static bool Command_FormatFixed3(int32_t raw_value,
                                 char *text,
                                 size_t text_size)
{
    int64_t value;
    int64_t integer_part;
    int64_t fractional_part;
    int written;

    if ((text == NULL) || (text_size == 0U))
    {
        return false;
    }

    value = raw_value;

    if (value < 0)
    {
        value = -value;

        integer_part = value / COMMAND_VALUE_SCALE;
        fractional_part = value % COMMAND_VALUE_SCALE;

        written = snprintf(text,
                           text_size,
                           "-%ld.%03ld",
                           (long)integer_part,
                           (long)fractional_part);
    }
    else
    {
        integer_part = value / COMMAND_VALUE_SCALE;
        fractional_part = value % COMMAND_VALUE_SCALE;

        written = snprintf(text,
                           text_size,
                           "%ld.%03ld",
                           (long)integer_part,
                           (long)fractional_part);
    }

    if ((written < 0) || ((size_t)written >= text_size))
    {
        return false;
    }

    return true;
}

/**
 * @brief 检查格式并分离指令名称、参数。
 *
 * 合法示例：
 * SETTEMPERATURE=25001
 * SETCIRCUT=100001
 *
 * 不合法示例：
 * SETTEMPERATURE
 * SETTEMPERATURE=25.001
 * SETTEMPERATURE=ABC
 * settemperature=25001
 */
CommandParseStatus_t Command_ParseLine(const char *line,
                                       CommandPacket_t *packet)
{
    const char *equal_pos;
    size_t name_len;
    size_t i;
    CommandParseStatus_t status;

    if ((line == NULL) || (packet == NULL) || (*line == '\0'))
    {
        return CMD_PARSE_EMPTY;
    }

    equal_pos = strchr(line, '=');

    if (equal_pos == NULL)
    {
        return CMD_PARSE_NO_EQUAL;
    }

    /* 只允许一个等号 */
    if (strchr(equal_pos + 1, '=') != NULL)
    {
        return CMD_PARSE_BAD_PARAMETER;
    }

    name_len = (size_t)(equal_pos - line);

    if (name_len == 0U)
    {
        return CMD_PARSE_BAD_NAME;
    }

    if (name_len >= COMMAND_NAME_MAX_LEN)
    {
        return CMD_PARSE_NAME_TOO_LONG;
    }

    /* 指令首字符必须是大写字母 */
    if ((line[0] < 'A') || (line[0] > 'Z'))
    {
        return CMD_PARSE_BAD_NAME;
    }

    for (i = 0U; i < name_len; i++)
    {
        if (!Command_IsValidNameChar(line[i]))
        {
            return CMD_PARSE_BAD_NAME;
        }
    }

    memcpy(packet->name, line, name_len);
    packet->name[name_len] = '\0';

    status = Command_ParseInt32(equal_pos + 1, &packet->parameter);

    return status;
}

/**
 * @brief 执行具体命令，并返回格式化后的成功回复。
 *
 * 新增命令时，只需在本函数中增加一个 else if 分支。
 */
CommandExecStatus_t Command_Execute(const CommandPacket_t *packet,
                                    char *reply,
                                    size_t reply_size)
{
    char value_text[20];
    int written;

    if ((packet == NULL) || (reply == NULL) || (reply_size == 0U))
    {
        return CMD_EXEC_REPLY_BUFFER_SMALL;
    }

    /*
     * 温度设置命令：
     * SETTEMPERATURE=25001
     */
    if (strcmp(packet->name, "SETTEMPERATURE") == 0)
    {
        if ((packet->parameter < TEMPERATURE_MIN_mC) ||
            (packet->parameter > TEMPERATURE_MAX_mC))
        {
            return CMD_EXEC_OUT_OF_RANGE;
        }

        g_target_temperature_mC = packet->parameter;

        if (!Command_FormatFixed3(g_target_temperature_mC,
                                  value_text,
                                  sizeof(value_text)))
        {
            return CMD_EXEC_REPLY_BUFFER_SMALL;
        }

        written = snprintf(reply,
                           reply_size,
                           "SETTEMPERATURE=%s OK\r\n",
                           value_text);
    }
    /*
     * 电流设置命令。
     * 注意：此处按需求保留 SETCIRCUT 拼写。
     */
    else if (strcmp(packet->name, "SETCIRCUT") == 0)
    {
        if ((packet->parameter < CURRENT_MIN_uA) ||
            (packet->parameter > CURRENT_MAX_uA))
        {
            return CMD_EXEC_OUT_OF_RANGE;
        }

        g_target_current_uA = packet->parameter;

        if (!Command_FormatFixed3(g_target_current_uA,
                                  value_text,
                                  sizeof(value_text)))
        {
            return CMD_EXEC_REPLY_BUFFER_SMALL;
        }

        written = snprintf(reply,
                           reply_size,
                           "SETCIRCUT=%s OK\r\n",
                           value_text);
    }
    
		/*
     * DAC 设置命令：
     * SETDAC=2500
     *
     * 默认设置 DAC8568 A 通道，也就是通道 0。
     * DAC8568 是 16 位 DAC，允许范围 0~65535。
     */
		
    else if (strcmp(packet->name, "SETDAC0") == 0)
    {
        float dac_voltage;

    /* 参数单位为 mV，DAC 输出范围为 0～5000 mV */
    if ((packet->parameter < DAC_MIN_mV) ||
        (packet->parameter > DAC_MAX_mV))
    {
        return CMD_EXEC_OUT_OF_RANGE;
    }

    /*
     * 例如：
     * packet->parameter = 1500
     * dac_voltage = 1.500V
     */
    dac_voltage = (float)packet->parameter / 1000.0f;
		
		if (!Command_FormatFixed3(dac_voltage,
                              value_text,
                              sizeof(value_text)))
        {
            return CMD_EXEC_REPLY_BUFFER_SMALL;
        }

    /*
     * 真正通过 SPI 写入 DAC8568，
     * 使 DAC0 引脚实际输出设置的电压。
     */
		PID_Control_SetManual(&TEC1_Control);
    DAC8568_SetVoltage(OutA, dac_voltage);

    written = snprintf(reply,
                       reply_size,
                       "SETDAC0=%ld.%03ldV OK\r\n",
                       (long)(packet->parameter / 1000),
                       (long)(packet->parameter % 1000));
			}
		
			else if (strcmp(packet->name, "SETDAC2") == 0)
    {
        float dac_voltage;

    /* 参数单位为 mV，DAC 输出范围为 0～5000 mV */
    if ((packet->parameter < DAC_MIN_mV) ||
        (packet->parameter > DAC_MAX_mV))
    {
        return CMD_EXEC_OUT_OF_RANGE;
    }

    /*
     * 例如：
     * packet->parameter = 1500
     * dac_voltage = 1.500V
     */
    dac_voltage = (float)packet->parameter / 1000.0f;
		
		if (!Command_FormatFixed3(dac_voltage,
                              value_text,
                              sizeof(value_text)))
        {
            return CMD_EXEC_REPLY_BUFFER_SMALL;
        }

    /*
     * 真正通过 SPI 写入 DAC8568，
     * 使 DAC0 引脚实际输出设置的电压。
     */
    DAC8568_SetVoltage(OutC, dac_voltage);

    written = snprintf(reply,
                       reply_size,
                       "SETDAC2=%ld.%03ldV OK\r\n",
                       (long)(packet->parameter / 1000),
                       (long)(packet->parameter % 1000));
			}

		
		else
    {
        return CMD_EXEC_UNKNOWN_COMMAND;
    }

    if ((written < 0) || ((size_t)written >= reply_size))
    {
        return CMD_EXEC_REPLY_BUFFER_SMALL;
    }

    return CMD_EXEC_OK;
}