#ifndef __COMMAND_PARSER_H
#define __COMMAND_PARSER_H

#include <stdint.h>
#include <stddef.h>

/* 指令名最大长度，不包括字符串结束符 '\0' */
#define COMMAND_NAME_MAX_LEN      24U

/* 温度、电流均使用千分之一单位 */
#define COMMAND_VALUE_SCALE       1000L

/*
 * 全局目标值：
 * g_target_temperature_mC：
 *     单位为 0.001 摄氏度。
 *     例如 25001 表示 25.001 ℃。
 *
 * g_target_current_uA：
 *     单位为 0.001 mA，也可理解为 uA。
 *     例如 100001 表示 100.001 mA。
 */
extern volatile int32_t g_target_temperature_mC;
extern volatile int32_t g_target_current_uA;




/* 一条解析完成后的指令 */
typedef struct
{
    char    name[COMMAND_NAME_MAX_LEN];
    int32_t parameter;
} CommandPacket_t;

/* 指令格式检查结果 */
typedef enum
{
    CMD_PARSE_OK = 0,
    CMD_PARSE_EMPTY,
    CMD_PARSE_NO_EQUAL,
    CMD_PARSE_BAD_NAME,
    CMD_PARSE_NAME_TOO_LONG,
    CMD_PARSE_BAD_PARAMETER,
    CMD_PARSE_PARAMETER_OVERFLOW
} CommandParseStatus_t;

/* 指令执行结果 */
typedef enum
{
    CMD_EXEC_OK = 0,
    CMD_EXEC_UNKNOWN_COMMAND,
    CMD_EXEC_OUT_OF_RANGE,
    CMD_EXEC_REPLY_BUFFER_SMALL
} CommandExecStatus_t;

/**
 * @brief  检查指令格式，并分离出“指令名”和“参数”。
 *
 * @param  line      接收到的一行字符串，例如 "SETTEMPERATURE=25001"
 * @param  packet    输出解析结果
 *
 * @return CommandParseStatus_t
 *
 * 使用示例：
 * CommandPacket_t packet;
 * if (Command_ParseLine("SETTEMPERATURE=25001", &packet) == CMD_PARSE_OK)
 * {
 *     // packet.name = "SETTEMPERATURE"
 *     // packet.parameter = 25001
 * }
 */
CommandParseStatus_t Command_ParseLine(const char *line,
                                       CommandPacket_t *packet);

/**
 * @brief  根据指令名和参数，设置对应全局变量，并生成成功回复。
 *
 * @param  packet          已解析完成的指令
 * @param  reply           回复字符串缓存
 * @param  reply_size      回复缓存长度
 *
 * @return CommandExecStatus_t
 *
 * 成功时 reply 示例：
 * "SETTEMPERATURE=25.001 OK\r\n"
 */
CommandExecStatus_t Command_Execute(const CommandPacket_t *packet,
                                    char *reply,
                                    size_t reply_size);

#endif