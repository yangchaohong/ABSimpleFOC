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
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc.h"       // 引入 FOC 控制头文件
#include "mt6816.h"    // 引入 MT6816 编码器驱动头文件
#include "my_cordic.h" // 引入 CORDIC 模块头文件
#include "ws2812b.h"   // 引入 WS2812B 驱动头文件
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_VREF 3.3f
#define ADC_RESOLUTION 4096.0f
#define SHUNT_RESISTANCE 0.010f
#define OPAMP_GAIN 20.0f
#define CURRENT_SENSITIVITY (SHUNT_RESISTANCE * OPAMP_GAIN) // 0.200 V/A

#define RX_BUF_SIZE 64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 串口命令缓冲区
static uint8_t rx_byte;
static char rx_line[RX_BUF_SIZE];
static uint8_t rx_idx = 0;

MT6816_Info_t my_encoder; // 编码器结构体实例
struct FOC_Motor motor = {
    .v_bus = 12.0f,     // 设置母线电压
    .pwm_period = 6400, // 设置 PWM 周期计数值
    .pole_pairs = 7,    // 设置电机极对数
}; // 电机结构体实例
struct PID_Controller pid_id = {
    .integral = 0.0f,
    .Kp = 2.0f,
    .Ki = 100.0f,
    .Kd = 0.0f,
    .integral_limit = 5.0f, // 积分抗饱和限幅
    .out_limit = 6.9f,      // 输出限幅
}; // 直轴电流环 PID 控制器实例
struct PID_Controller pid_iq = {
    .integral = 0.0f,
    .Kp = 2.0f,
    .Ki = 100.0f,
    .Kd = 0.0f,
    .integral_limit = 5.0f, // 积分抗饱和限幅
    .out_limit = 6.9f,      // 输出限幅
}; // 交轴电流环 PID 控制器
bool isProtecting = false; // 过流保护标志
bool isRunning = false;    // 电机使能状态标志

// ADC 零点偏置值
float adc_offset_u = 0.0f;
float adc_offset_v = 0.0f;
float adc_offset_w = 0.0f;
float adc_offset_bus = 0.0f;

bool inited = false;
bool adc_calibrated = false;
int calibration_count = 0;

float gain_u = 1.0f;
float gain_v = 1.0f;
float gain_w = 1.0f;
float gain_bus = 1.0f;

volatile float debug_Iu, debug_Iv, debug_Iw, debug_Ibus, debug_Vin;
volatile float debug_angle;
volatile uint32_t debug_raw_u, debug_raw_v, debug_raw_w, debug_raw_bus,
    debug_raw_vin;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void FOC_Calib();
char *f2s(float val, int decimals);
void Set_PWM();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_CORDIC_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  printf("\r\nSystem Initialized. Starting main loop...\r\n");
  cordic_init_config(); /* 配置 CORDIC 为 SINE 模式，双输出 */
  __HAL_TIM_MOE_ENABLE(&htim8);
  WS2812_Init();

  // 6. 开启高级定时器总闸
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start(&htim1);

  FOC_Calib(); // 执行 FOC 校准，确定电机旋转方向和零点偏移角

  // 3. 初始化并校准 ADC1 & ADC2
  adc_offset_u = adc_offset_v = adc_offset_w = 2048.0f; // 初始假设零点在中间值
  adc_offset_bus = 0.0f;
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_InjectedStart(&hadc2); // 从机 ADC2：启动注入转换（响应 TIM1 TRGO2）
  HAL_ADCEx_InjectedStart_IT(&hadc1); // 主机 ADC1：启动注入转换并开启中断

  // 7. 开启串口接收中断（单字节模式）
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    uint32_t raw_v = HAL_ADCEx_InjectedGetValue(
        &hadc1, ADC_INJECTED_RANK_1); // ADC1_IN2 (V相)
    uint32_t raw_bus = HAL_ADCEx_InjectedGetValue(
        &hadc1, ADC_INJECTED_RANK_2); // ADC1_IN15 (Bus)
    uint32_t raw_vin = HAL_ADCEx_InjectedGetValue(
        &hadc1, ADC_INJECTED_RANK_3); // ADC1_IN15 (Bus)
    uint32_t raw_u = HAL_ADCEx_InjectedGetValue(
        &hadc2, ADC_INJECTED_RANK_1); // ADC2_IN1 (U相)
    uint32_t raw_w = HAL_ADCEx_InjectedGetValue(
        &hadc2, ADC_INJECTED_RANK_2); // ADC2_IN4 (W相)}

    debug_raw_u = raw_u;
    debug_raw_v = raw_v;
    debug_raw_w = raw_w;
    debug_raw_bus = raw_bus;
    debug_raw_vin = raw_vin;

    motor.Ia = ((float)raw_u - adc_offset_u) / ADC_RESOLUTION * ADC_VREF /
               (CURRENT_SENSITIVITY * gain_u);
    motor.Ib = ((float)raw_v - adc_offset_v) / ADC_RESOLUTION * ADC_VREF /
               (CURRENT_SENSITIVITY * gain_v);
    motor.Ic = ((float)raw_w - adc_offset_w) / ADC_RESOLUTION * ADC_VREF /
               (CURRENT_SENSITIVITY * gain_w);
    float Ibus_temp = ((float)raw_bus - adc_offset_bus) / ADC_RESOLUTION *
                      ADC_VREF / (CURRENT_SENSITIVITY * gain_bus);
    float Ivin_temp = raw_vin * 0.0f; // 预留 Vin 电压测量，暂未使用

    debug_Iu = motor.Ia;
    debug_Iv = motor.Ib;
    debug_Iw = motor.Ic;
    debug_Ibus = Ibus_temp;
    debug_Vin = Ivin_temp;

    MT6816_ReadAngle(&my_encoder);
    debug_angle = my_encoder.angle_rad * 180.0f;
    motor.mech_angle = my_encoder.angle_rad;
    motor.elec_angle = Normalize_Angle_0_to_2PI(
        motor.direction * motor.mech_angle * motor.pole_pairs -
        motor.zero_offset);

    FOC_Run(&motor, &pid_id, &pid_iq); // 执行 FOC 控制算法

    Set_PWM(); // 更新 PWM 输出占空比
  }
}

void FOC_Calib() {
  // 1. 设置电机为开环模式，施加一个固定的电压向量

  OPENLOOP_Run(&motor, 3.5f, 0.0f); // 施加固定的直轴电压，电角度为 0
  Set_PWM();                        // 更新 PWM 输出占空比
  HAL_Delay(500);
  MT6816_ReadAngle(&my_encoder);
  float rad_start = my_encoder.angle_rad;

  OPENLOOP_Run(&motor, 3.5f, 1.0f); // 施加固定的直轴电压，电角度为 1
  Set_PWM();                        // 更新 PWM 输出占空比
  HAL_Delay(500);
  MT6816_ReadAngle(&my_encoder);
  float rad_end = my_encoder.angle_rad;

  if (rad_end > rad_start)
    motor.direction = 1; // 正转
  else
    motor.direction = -1; // 反转

  OPENLOOP_Run(&motor, 3.5f, 0.0f); // 施加固定的直轴电压，电角度为 0
  Set_PWM();                        // 更新 PWM 输出占空比
  HAL_Delay(500);
  MT6816_ReadAngle(&my_encoder);
  float theta_mech = my_encoder.angle_rad;

  motor.zero_offset =
      Normalize_Angle_0_to_2PI(motor.direction * theta_mech * motor.pole_pairs);
  printf("Calibration complete. Direction: %s, Zero Offset: %s rad\r\n",
         motor.direction > 0 ? "Positive" : "Negative",
         f2s(motor.zero_offset, 4));

  OPENLOOP_Run(&motor, 0.0f, 0.0f);
  HAL_Delay(100);
}

char *f2s(float val, int decimals) {
  static char buffers[4][20];
  static int ring_idx = 0;
  char *buf = buffers[ring_idx];
  ring_idx = (ring_idx + 1) % 4;

  if (isnan(val)) {
    strcpy(buf, "NaN");
    return buf;
  }
  if (isinf(val)) {
    strcpy(buf, val > 0 ? "Inf" : "-Inf");
    return buf;
  }

  char *p = buf;
  if (val < 0.0f) {
    *p++ = '-';
    val = -val;
  }
  float rounding = 0.5f;
  for (int i = 0; i < decimals; i++)
    rounding /= 10.0f;
  val += rounding;

  int ipart = (int)val;
  p += sprintf(p, "%d", ipart);
  if (decimals > 0) {
    *p++ = '.';
    float fpart_f = val - (float)ipart;
    for (int i = 0; i < decimals; i++) {
      fpart_f *= 10.0f;
      int digit = (int)fpart_f;
      if (digit < 0)
        digit = 0;
      if (digit > 9)
        digit = 9;
      *p++ = '0' + digit;
      fpart_f -= (float)digit;
    }
  }
  *p = '\0';
  return buf;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    if (rx_byte == '\n' || rx_byte == '\r') {
      if (rx_idx > 0) {
        rx_line[rx_idx] = '\0';
        Commander_Run(rx_line);
        rx_idx = 0;
      }
    } else {
      if (rx_idx < RX_BUF_SIZE - 1) {
        rx_line[rx_idx++] = (char)rx_byte;
      }
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

void Set_PWM() {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor.Ta);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor.Tb);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor.Tc);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
    number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
    file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
