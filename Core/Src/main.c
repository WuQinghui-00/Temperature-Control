/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "command_parser.h"
#include "uart1_comm.h"
#include "bsp_DAC8568.h"
#include "thermal.h"
#include "formula.h"
#include "ads1255.h"
#include "pid.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_FILTER_N          10U
#define ADS1256_INVALID_CODE  ((int32_t)0x7FFFFFFF)

#define TEC1_TARGET_TEMPERATURE      25.0f
#define TEC1_CONTROL_PERIOD_MS       200U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static int32_t ntc_adc_code = 0;
static double ntc_diff_voltage = 0.0f;
static double adc_ntc_node_voltage = 0.0;
static double ntc_temperature = 0.0f;
static double ntc_resistance = 0.0f;

static double dac_temperature = 0.0f;
static double dac_ntc_resistance = 0.0f;
static uint8_t dac_temp_valid = 0U;

static uint8_t ntc_data_valid = 0U;
static uint32_t ntc_raw24 = 0U;

static PID TEC1_PID;
PID_Control TEC1_Control;
/* USER CODE BEGIN PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief 处理一条来自电脑的完整命指令
 *
 * ??????????,????????????,
 * ????????????
 */
static void App_ProcessPcCommand(void)
{
    char rx_line[UART1_RX_LINE_MAX_LEN];
    char tx_reply[UART1_TX_BUFFER_SIZE];

    CommandPacket_t command;
    CommandParseStatus_t parse_status;
    CommandExecStatus_t exec_status;

    /*
     * ?????? DMA ????????,
     * ?????????,???????????
     */
    if (UART1_Comm_IsTxBusy())
    {
        return;
    }

    if (!UART1_Comm_ReadLine(rx_line, sizeof(rx_line)))
    {
        return;
    }

    /* ???????:????,??????? */
    parse_status = Command_ParseLine(rx_line, &command);

    if (parse_status != CMD_PARSE_OK)
    {
        (void)UART1_SendString_DMA("ERROR FORMAT\r\n");
        return;
    }

    /*
     * ???????:
     * 根据指令名和参数设置目标变量
     * ?????????
     */
    exec_status = Command_Execute(&command,
                                  tx_reply,
                                  sizeof(tx_reply));

    switch (exec_status)
    {
        case CMD_EXEC_OK:
            (void)UART1_SendString_DMA("%s", tx_reply);
            break;

        case CMD_EXEC_UNKNOWN_COMMAND:
            (void)UART1_SendString_DMA("ERROR UNKNOWN COMMAND\r\n");
            break;

        case CMD_EXEC_OUT_OF_RANGE:
            (void)UART1_SendString_DMA("ERROR OUT OF RANGE\r\n");
            break;

        default:
            (void)UART1_SendString_DMA("ERROR INTERNAL\r\n");
            break;
    }
}


/* ?32???????????????? */
static void Uint32_ToBinaryString(uint32_t value, char output[33])
{
    for (uint32_t i = 0U; i < 32U; i++)
    {
        uint32_t bit_pos = 31U - i;

        output[i] = ((value >> bit_pos) & 0x01U) ? '1' : '0';
    }

    output[32] = '\0';
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  
  UART1_Comm_Start();
	
	DAC8568_Init();
	
	/* DAC0 -> AIN0 */
	DAC8568_SetVoltage(OutA,1.25);
	
	/* DAC2 -> AIN1 */
  DAC8568_SetVoltage(OutC, 1.25);
	
	ADS1256_GPIO_Init();
  ADS1256_CfgADC(PGA_1, DATARATE_50);
	
	//设置差分通道
	ADS1256_SetDiffChannel(POSITIVE_AIN0, NEGTIVE_AIN1);
	
	Formula_TecPid_Init(&TEC1_PID,
                        0.8f,     // Kp
                        0.02f,    // Ki
                        0.1f,     // Kd
                        25.0f);   // 初始温度


  if (!PID_Control_Init(&TEC1_Control,
                      &TEC1_PID,
                      TEC1_TARGET_TEMPERATURE,
                      TEC1_CONTROL_PERIOD_MS,
                      TEC_CHANNEL_1))
{
    Error_Handler();
}

/* 上电获得有效温度自动整定一次 */
PID_Control_RequestAutoTune(&TEC1_Control);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	uint32_t last_send_tick = 0U;
	uint16_t dac0_code;
	uint16_t dac2_code;
	
  double dac0_v;
  double dac2_v;
  double dac_diff_v;
	
  while (1)
  {
    /* USER CODE END WHILE */
		
		App_ProcessPcCommand();
    
		
		 dac0_code = g_dac8568_code[0];
     dac2_code = g_dac8568_code[2];

     dac0_v = ((double)dac0_code * 5.0f) / 65535.0f;
     dac2_v = ((double)dac2_code * 5.0f) / 65535.0f;

     dac_diff_v = dac0_v - dac2_v;
		
		//读取adc电压差分数据
	  ntc_adc_code =
    ADS1256_GetAdcTrimmedMean1(ADC_FILTER_N);
		
		ntc_data_valid = 0U;
    ntc_raw24 = 0U;
    ntc_diff_voltage = 0.0;
    adc_ntc_node_voltage = 0.0;


    //判断adc返回值
    if (ntc_adc_code != ADS1256_INVALID_CODE)
{
    ntc_raw24 =
        ((uint32_t)ntc_adc_code) &
        0x00FFFFFFU;

	  //adc测量差分
    ntc_diff_voltage =
            ADS1256_CodeToVoltage(
            ntc_adc_code,
            2.5,
            PGA_1
        );

	  //还原节点N1电压
	  adc_ntc_node_voltage =
            ntc_diff_voltage +
            dac2_v;
	
						
				//差分电压转换成温度和电阻（真实采集到的，可用于计算真实温度）
				ntc_data_valid =
    NTC_NodeVoltageToTemperature(
        adc_ntc_node_voltage,
        &ntc_temperature,
        &ntc_resistance
    );

  }

	//pid控制
	PID_Control_Task(&TEC1_Control,
                     (float)ntc_temperature,
                     ntc_data_valid,
                     HAL_GetTick());
		
		
		//根据理论差分电压计算模拟温度（DAC输出电压转换的理论温度）
		 dac_temp_valid =
    NTC_NodeVoltageToTemperature(
        dac0_v,
        &dac_temperature,
        &dac_ntc_resistance
    );


    /*
     * ????????
     */
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
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief USART DMA ???????
 *
 * RX DMA ?? Circular ???,???????????
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    UART1_Comm_OnRxDmaEvent(huart);
}

/**
 * @brief USART DMA ???????
 *
 * RX DMA ?? Circular ???,?????????????
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART1_Comm_OnRxDmaEvent(huart);
}

/**
 * @brief USART DMA ???????
 *
 * ?????????????
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UART1_Comm_OnTxCplt(huart);
}

/**
 * @brief USART ?????
 *
 * ???????????????,
 * ???? UART DMA ???
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART1_Comm_OnError(huart);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
