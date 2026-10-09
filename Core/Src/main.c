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
#include <stdlib.h>
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
// 串口命令缓冲区 (异步处理，主循环执行)
static uint8_t rx_byte;
static char rx_line[RX_BUF_SIZE];
static uint8_t rx_idx = 0;
static char rx_cmd_buf[RX_BUF_SIZE];
static volatile bool rx_cmd_ready = false;

// 运动控制模式与目标值 (SimpleFOC 架构)
MotionControlType controller_type = MOTION_TORQUE; // 默认力矩模式
TorqueControlType torque_type = TORQUE_VOLTAGE;    // 默认电压等效模式 (防电流环未校准饱和)
float target = 0.0f; // 当前模式目标值 (M 指令设置)

// 限幅参数
float velocity_limit = 50.0f; // 最大转速 (rad/s)
float voltage_limit = 6.9f;   // 最大相电压 (V)
float current_limit = 2.0f;   // 最大力矩电流 (A)

MT6816_Info_t my_encoder; // 编码器结构体实例
struct FOC_Motor motor = {
    .v_bus = 12.0f,     // 设置母线电压
    .pwm_period = 6400, // 设置 PWM 周期计数值
    .pole_pairs = 7,    // 设置电机极对数
}; // 电机结构体实例

// 电流环 PID
struct PID_Controller pid_id = {
    .integral = 0.0f,
    .Kp = 0.5f,
    .Ki = 100.0f,
    .Kd = 0.0f,
    .integral_limit = 5.0f, // 积分抗饱和限幅
    .out_limit = 4.0f,      // 输出限幅
    .output_ramp = 0.0f,
};
struct PID_Controller pid_iq = {
    .integral = 0.0f,
    .Kp = 0.5f,
    .Ki = 100.0f,
    .Kd = 0.0f,
    .integral_limit = 5.0f, // 积分抗饱和限幅
    .out_limit = 4.0f,      // 输出限幅
    .output_ramp = 0.0f,
};

// 速度环 PID
struct PID_Controller pid_vel = {
    .integral = 0.0f,
    .Kp = 0.2f,
    .Ki = 1.0f,
    .Kd = 0.0f,
    .integral_limit = 2.0f,
    .out_limit = 2.0f, // 输出交轴电流限幅 (A)
    .output_ramp = 0.0f,
};

// 位置环 PID / P控制器
struct PID_Controller pid_angle = {
    .integral = 0.0f,
    .Kp = 20.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .integral_limit = 50.0f,
    .out_limit = 50.0f, // 输出目标速度限幅 (rad/s)
    .output_ramp = 0.0f,
};

// 低通滤波器
struct LowPassFilter lpf_velocity = {
    .Tf = 0.005f,
    .prev_out = 0.0f,
};
struct LowPassFilter lpf_angle = {
    .Tf = 0.0f,
    .prev_out = 0.0f,
};

// 数据监视器配置 (SimpleFOC Studio Monitor)
// 位掩码: Bit6=Target, Bit5=Vq, Bit4=Vd, Bit3=Cq, Bit2=Cd, Bit1=Vel, Bit0=Angle
#define MON_TARGET (1 << 6)
#define MON_VOLT_Q (1 << 5)
#define MON_VOLT_D (1 << 4)
#define MON_CURR_Q (1 << 3)
#define MON_CURR_D (1 << 2)
#define MON_VEL (1 << 1)
#define MON_ANGLE (1 << 0)

int monitor_downsample = 0;            // 0=关闭，>0 表示下采样周期 (ms)
uint8_t monitor_variables = 0b1111111; // 7位变量掩码，默认全选
static uint32_t monitor_timer = 0;

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
void Motion_Control_Loop(float dt);
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

  // 开启三相 MOS 驱动使能 (PA4, PA5, PA6: DISB1/2/3 拉高使能)
  HAL_GPIO_WritePin(GPIOA, DISB1_Pin | DISB2_Pin | DISB3_Pin, GPIO_PIN_SET);

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
    /* 1. 处理串口 Command 交互 (在主循环执行，避免阻塞中断) */
    if (rx_cmd_ready) {
      Commander_Run(rx_cmd_buf);
      rx_cmd_ready = false;
    }

    /* 2. 外环运动控制更新 (按 1ms / 1kHz 周期调度) */
    static uint32_t motion_timer = 0;
    uint32_t now = HAL_GetTick();
    if (now - motion_timer >= 1) {
      float dt = (float)(now - motion_timer) * 0.001f;
      motion_timer = now;
      Motion_Control_Loop(dt);
    }

    /* 3. 数据监视器波形流发送 (SimpleFOC Studio 实时波形 Telemetry Stream) */
    if (monitor_downsample > 0 && monitor_variables != 0) {
      if (now - monitor_timer >= (uint32_t)monitor_downsample) {
        monitor_timer = now;
        bool first = true;
        // Bit 6: Target
        if (monitor_variables & MON_TARGET) {
          if (!first)
            printf("\t");
          printf("%s", f2s(target, 4));
          first = false;
        }
        // Bit 5: Voltage Q
        if (monitor_variables & MON_VOLT_Q) {
          if (!first)
            printf("\t");
          printf("%s", f2s(motor.Vq, 4));
          first = false;
        }
        // Bit 4: Voltage D
        if (monitor_variables & MON_VOLT_D) {
          if (!first)
            printf("\t");
          printf("%s", f2s(motor.Vd, 4));
          first = false;
        }
        // Bit 3: Current Q
        if (monitor_variables & MON_CURR_Q) {
          if (!first)
            printf("\t");
          printf("%s", f2s(motor.Iq, 4));
          first = false;
        }
        // Bit 2: Current D
        if (monitor_variables & MON_CURR_D) {
          if (!first)
            printf("\t");
          printf("%s", f2s(motor.Id, 4));
          first = false;
        }
        // Bit 1: Velocity
        if (monitor_variables & MON_VEL) {
          if (!first)
            printf("\t");
          printf("%s", f2s(motor.velocity, 4));
          first = false;
        }
        // Bit 0: Angle
        if (monitor_variables & MON_ANGLE) {
          if (!first)
            printf("\t");
          printf("%s", f2s(motor.mech_angle, 4));
          first = false;
        }
        if (!first) {
          printf("\r\n");
        }
      }
    }
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
    if (!isRunning) {
      uint32_t half_arr = (uint32_t)(motor.pwm_period / 2);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, half_arr);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, half_arr);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, half_arr);
      return;
    }

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
  Set_PWM();
  HAL_Delay(100);
}

char *f2s(float val, int decimals) {
  static char buffers[8][20];
  static int ring_idx = 0;
  char *buf = buffers[ring_idx];
  ring_idx = (ring_idx + 1) % 8;

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
        if (!rx_cmd_ready) {
          strncpy(rx_cmd_buf, rx_line, RX_BUF_SIZE - 1);
          rx_cmd_buf[RX_BUF_SIZE - 1] = '\0';
          rx_cmd_ready = true;
        }
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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_READY;
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

void Set_PWM() {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)motor.Ta);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)motor.Tb);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)motor.Tc);
}

void Motion_Control_Loop(float dt) {
  // 1. 估算转速并做一阶低通滤波
  static float prev_angle = 0.0f;
  static bool angle_inited = false;
  if (!angle_inited) {
    prev_angle = motor.mech_angle;
    angle_inited = true;
  }

  float d_angle = motor.mech_angle - prev_angle;
  // 核心修复：系统角度范围为 [0, 2.0)，半圈为 1.0f，一整圈为 2.0f！
  // 必须以 1.0f 和 2.0f 进行跨圈跳变补偿，杜绝在 2.0 边界处产生暴跌/暴冲毛刺！
  if (d_angle > 1.0f)
    d_angle -= 2.0f;
  else if (d_angle < -1.0f)
    d_angle += 2.0f;
  prev_angle = motor.mech_angle;

  // 转换为标准物理角速度 (rad/s)，由于 2.0 代表 2*PI 弧度，故乘以 PI
  float raw_vel = (dt > 0.0f) ? (d_angle * PI / dt) : 0.0f;
  float alpha = lpf_velocity.Tf / (lpf_velocity.Tf + dt);
  motor.velocity = alpha * motor.velocity + (1.0f - alpha) * raw_vel;

  if (!isRunning)
    return;

  // 2. 根据当前控制模式执行外环级联计算
  switch (controller_type) {
  case MOTION_TORQUE:
    // 直接力矩模式：target 视为目标
    motor.target_Id = 0.0f;
    motor.target_Iq = target;
    break;

  case MOTION_VELOCITY: {
    // 速度闭环：target 视为目标速度 (rad/s)
    float vel_err = target - motor.velocity;
    float cmd_iq = PID_Compute(&pid_vel, vel_err);
    motor.target_Id = 0.0f;
    motor.target_Iq = cmd_iq;
    break;
  }

  case MOTION_ANGLE: {
    // 位置闭环伺服：就近最短路径计算误差 (同样以 1.0f/2.0f 归一化)
    float pos_err = target - motor.mech_angle;
    if (pos_err > 1.0f)
      pos_err -= 2.0f;
    else if (pos_err < -1.0f)
      pos_err += 2.0f;

    float target_vel = PID_Compute(&pid_angle, pos_err);
    // 目标速度限幅
    target_vel = fmaxf(fminf(target_vel, velocity_limit), -velocity_limit);
    // 级联速度环 PID
    float vel_err = target_vel - motor.velocity;
    float cmd_iq = PID_Compute(&pid_vel, vel_err);
    motor.target_Id = 0.0f;
    motor.target_Iq = cmd_iq;
    break;
  }

  case MOTION_VELOCITY_OPENLOOP:
  case MOTION_ANGLE_OPENLOOP:
    break;
  }
}

/**
 * @brief  SimpleFOC Studio 协议解析器 (支持带 'M' 前缀及原生命令输入)
 * @param  cmd: 接收到的命令字符串
 */
void Commander_Run(char *cmd) {
  if (cmd == NULL)
    return;

  // 1. 去除前导空白字符
  while (*cmd == ' ' || *cmd == '\t' || *cmd == '\r' || *cmd == '\n') {
    cmd++;
  }
  if (*cmd == '\0')
    return;

  // 2. 去除尾部换行与空白
  int len = (int)strlen(cmd);
  while (len > 0 && (cmd[len - 1] == '\r' || cmd[len - 1] == '\n' ||
                     cmd[len - 1] == ' ' || cmd[len - 1] == '\t')) {
    cmd[--len] = '\0';
  }
  if (len == 0)
    return;

  // 3. 支持大小写 'M' / 'm' 前缀 (若有则跳过，无亦可兼容)
  char *p = cmd;
  if (*p == 'M' || *p == 'm') {
    p++;
  }

  // 4. 目标值设定与查询 (M<val> 或 纯数字 <val>)
  if (*p == '\0' || (*p >= '0' && *p <= '9') || *p == '-' || *p == '+' ||
      *p == '.') {
    if (*p != '\0') {
      target = atof(p);
    }
    printf("%s\r\n", f2s(target, 4));
    return;
  }

  // 5. 运动模式设置与查询 (MC<val> 或 C<val>)
  if (p[0] == 'C' || p[0] == 'c') {
    if (p[1] != '\0') {
      controller_type = (MotionControlType)atoi(&p[1]);
    }
    switch (controller_type) {
    case MOTION_TORQUE:
      printf("Motion:torque\r\n");
      break;
    case MOTION_VELOCITY:
      printf("Motion:vel\r\n");
      break;
    case MOTION_ANGLE:
      printf("Motion:angle\r\n");
      break;
    case MOTION_VELOCITY_OPENLOOP:
      printf("Motion:vel open\r\n");
      break;
    case MOTION_ANGLE_OPENLOOP:
      printf("Motion:angle open\r\n");
      break;
    default:
      printf("Motion:unknown\r\n");
      break;
    }
    return;
  }

  // 6. 扭矩类型设置与查询 (MT<val> 或 T<val>)
  if (p[0] == 'T' || p[0] == 't') {
    if (p[1] != '\0') {
      torque_type = (TorqueControlType)atoi(&p[1]);
    }
    switch (torque_type) {
    case TORQUE_VOLTAGE:
      printf("Torque:volt\r\n");
      break;
    case TORQUE_DC_CURRENT:
      printf("Torque:dc curr\r\n");
      break;
    case TORQUE_FOC_CURRENT:
      printf("Torque:foc curr\r\n");
      break;
    default:
      printf("Torque:volt\r\n");
      break;
    }
    return;
  }

  // 7. 电机使能与状态 (ME<0|1> 或 E<0|1>)
  if (p[0] == 'E' || p[0] == 'e') {
    if (p[1] != '\0') {
      int en = atoi(&p[1]);
      if (en == 1) {
        isRunning = true;
        HAL_GPIO_WritePin(GPIOA, DISB1_Pin | DISB2_Pin | DISB3_Pin, GPIO_PIN_SET);
      } else {
        isRunning = false;
        HAL_GPIO_WritePin(GPIOA, DISB1_Pin | DISB2_Pin | DISB3_Pin, GPIO_PIN_RESET);
        motor.target_Iq = 0.0f;
        motor.target_Id = 0.0f;
        pid_id.integral = 0.0f;
        pid_iq.integral = 0.0f;
        pid_vel.integral = 0.0f;
        pid_angle.integral = 0.0f;
        uint32_t half_arr = (uint32_t)(motor.pwm_period / 2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, half_arr);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, half_arr);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, half_arr);
      }
    }
    printf("Status:%d\r\n", isRunning ? 1 : 0);
    return;
  }

  // 8. 状态监视查询: MMG0 ~ MMG6 或 MG0 ~ MG6
  if ((p[0] == 'M' || p[0] == 'm') && (p[1] == 'G' || p[1] == 'g')) {
    p += 1; // 偏移到 G...
  }
  if (p[0] == 'G' || p[0] == 'g') {
    int idx = atoi(&p[1]);
    switch (idx) {
    case 0:
      printf("Monitor | target:%s\r\n", f2s(target, 4));
      break;
    case 1:
      printf("Monitor | Vq:%s\r\n", f2s(motor.Vq, 4));
      break;
    case 2:
      printf("Monitor | Vd:%s\r\n", f2s(motor.Vd, 4));
      break;
    case 3:
      printf("Monitor | Cq:%s\r\n", f2s(motor.Iq, 4));
      break;
    case 4:
      printf("Monitor | Cd:%s\r\n", f2s(motor.Id, 4));
      break;
    case 5:
      printf("Monitor | vel:%s\r\n", f2s(motor.velocity, 4));
      break;
    case 6:
      printf("Monitor | angle:%s\r\n", f2s(motor.mech_angle, 4));
      break;
    default:
      break;
    }
    return;
  }

  // 9. 监视器配置: MMD / MD, MMC / MC, MMS / MS
  char *mon_p = NULL;
  if ((p[0] == 'M' || p[0] == 'm') &&
      (p[1] == 'D' || p[1] == 'd' || p[1] == 'C' || p[1] == 'c' ||
       p[1] == 'S' || p[1] == 's')) {
    mon_p = &p[1]; // 跳过前缀 M，指向 'D', 'C', 'S'
  } else if (p[0] == 'D' || p[0] == 'd') {
    mon_p = &p[0];
  } else if (p[0] == 'S' || p[0] == 's') {
    // 排除 MSE (el. offset) 和 MSM (offset)
    if (p[1] != 'E' && p[1] != 'e' && p[1] != 'M' && p[1] != 'm') {
      mon_p = &p[0];
    }
  }

  if (mon_p != NULL) {
    // A. 下采样配置 MD / MMD
    if (mon_p[0] == 'D' || mon_p[0] == 'd') {
      char *arg = &mon_p[1];
      while (*arg == ' ')
        arg++;
      if (*arg != '\0') {
        monitor_downsample = atoi(arg);
      }
      printf("Monitor | downsample:%d\r\n", monitor_downsample);
      return;
    }

    // B. 清空监视变量 MC / MMC
    if (mon_p[0] == 'C' || mon_p[0] == 'c') {
      monitor_variables = 0;
      printf("Monitor | cleared\r\n");
      return;
    }

    // C. 设置或查询监视变量 MS / MMS
    if (mon_p[0] == 'S' || mon_p[0] == 's') {
      char *arg = &mon_p[1];
      while (*arg == ' ')
        arg++;
      if (*arg != '\0') {
        // 设置 7 位二进制掩码: Bit6=Target ... Bit0=Angle
        monitor_variables = 0;
        for (int i = 0; i < 7 && arg[i] != '\0'; i++) {
          if (arg[i] == '1') {
            monitor_variables |= (1 << (6 - i));
          }
        }
        printf("Monitor | set\r\n");
      } else {
        // 查询全部 7 项变量 (SimpleFOC all 格式，供上位机 DRO 显示)
        printf("Monitor | all:%s\t%s\t%s\t%s\t%s\t%s\t%s\r\n", f2s(target, 4),
               f2s(motor.Vq, 4), f2s(motor.Vd, 4), f2s(motor.Iq, 4),
               f2s(motor.Id, 4), f2s(motor.velocity, 4),
               f2s(motor.mech_angle, 4));
      }
      return;
    }
  }

  // 10. 限制 Limits: MLV/LV, MLU/LU, MLC/LC
  if (p[0] == 'L' || p[0] == 'l') {
    if (p[1] == 'V' || p[1] == 'v') {
      if (p[2] != '\0')
        velocity_limit = atof(&p[2]);
      printf("Limits|vel:%s\r\n", f2s(velocity_limit, 4));
      return;
    }
    if (p[1] == 'U' || p[1] == 'u') {
      if (p[2] != '\0') {
        voltage_limit = atof(&p[2]);
        pid_id.out_limit = voltage_limit;
        pid_iq.out_limit = voltage_limit;
      }
      printf("Limits|volt:%s\r\n", f2s(voltage_limit, 4));
      return;
    }
    if (p[1] == 'C' || p[1] == 'c') {
      if (p[2] != '\0') {
        current_limit = atof(&p[2]);
        pid_vel.out_limit = current_limit;
      }
      printf("Limits|curr:%s\r\n", f2s(current_limit, 4));
      return;
    }
  }

  // 11. 速度环调谐: MVP / VP...
  if (p[0] == 'V' || p[0] == 'v') {
    if (p[1] == 'P' || p[1] == 'p') {
      if (p[2] != '\0')
        pid_vel.Kp = atof(&p[2]);
      printf("PID vel|P: %s\r\n", f2s(pid_vel.Kp, 4));
      return;
    }
    if (p[1] == 'I' || p[1] == 'i') {
      if (p[2] != '\0')
        pid_vel.Ki = atof(&p[2]);
      printf("PID vel|I: %s\r\n", f2s(pid_vel.Ki, 4));
      return;
    }
    if (p[1] == 'D' || p[1] == 'd') {
      if (p[2] != '\0')
        pid_vel.Kd = atof(&p[2]);
      printf("PID vel|D: %s\r\n", f2s(pid_vel.Kd, 4));
      return;
    }
    if (p[1] == 'R' || p[1] == 'r') {
      if (p[2] != '\0')
        pid_vel.output_ramp = atof(&p[2]);
      printf("PID vel|ramp:%s\r\n", f2s(pid_vel.output_ramp, 4));
      return;
    }
    if (p[1] == 'L' || p[1] == 'l') {
      if (p[2] != '\0')
        pid_vel.out_limit = atof(&p[2]);
      printf("PID vel|limit:%s\r\n", f2s(pid_vel.out_limit, 4));
      return;
    }
    if (p[1] == 'F' || p[1] == 'f') {
      if (p[2] != '\0')
        lpf_velocity.Tf = atof(&p[2]);
      printf("PID vel|Tf: %s\r\n", f2s(lpf_velocity.Tf, 4));
      return;
    }
  }

  // 12. 角度环调谐: MAP / AP...
  if (p[0] == 'A' || p[0] == 'a') {
    if (p[1] == 'P' || p[1] == 'p') {
      if (p[2] != '\0')
        pid_angle.Kp = atof(&p[2]);
      printf("PID angle|P: %s\r\n", f2s(pid_angle.Kp, 4));
      return;
    }
    if (p[1] == 'I' || p[1] == 'i') {
      if (p[2] != '\0')
        pid_angle.Ki = atof(&p[2]);
      printf("PID angle|I: %s\r\n", f2s(pid_angle.Ki, 4));
      return;
    }
    if (p[1] == 'D' || p[1] == 'd') {
      if (p[2] != '\0')
        pid_angle.Kd = atof(&p[2]);
      printf("PID angle|D: %s\r\n", f2s(pid_angle.Kd, 4));
      return;
    }
    if (p[1] == 'R' || p[1] == 'r') {
      if (p[2] != '\0')
        pid_angle.output_ramp = atof(&p[2]);
      printf("PID angle|ramp:%s\r\n", f2s(pid_angle.output_ramp, 4));
      return;
    }
    if (p[1] == 'L' || p[1] == 'l') {
      if (p[2] != '\0')
        pid_angle.out_limit = atof(&p[2]);
      printf("PID angle|limit:%s\r\n", f2s(pid_angle.out_limit, 4));
      return;
    }
    if (p[1] == 'F' || p[1] == 'f') {
      if (p[2] != '\0')
        lpf_angle.Tf = atof(&p[2]);
      printf("PID angle|Tf: %s\r\n", f2s(lpf_angle.Tf, 4));
      return;
    }
  }

  // 13. 电流 Q 轴调谐: MQP / QP...
  if (p[0] == 'Q' || p[0] == 'q') {
    if (p[1] == 'P' || p[1] == 'p') {
      if (p[2] != '\0')
        pid_iq.Kp = atof(&p[2]);
      printf("PID curr q|P: %s\r\n", f2s(pid_iq.Kp, 4));
      return;
    }
    if (p[1] == 'I' || p[1] == 'i') {
      if (p[2] != '\0')
        pid_iq.Ki = atof(&p[2]);
      printf("PID curr q|I: %s\r\n", f2s(pid_iq.Ki, 4));
      return;
    }
    if (p[1] == 'D' || p[1] == 'd') {
      if (p[2] != '\0')
        pid_iq.Kd = atof(&p[2]);
      printf("PID curr q|D: %s\r\n", f2s(pid_iq.Kd, 4));
      return;
    }
    if (p[1] == 'R' || p[1] == 'r') {
      if (p[2] != '\0')
        pid_iq.output_ramp = atof(&p[2]);
      printf("PID curr q|ramp:%s\r\n", f2s(pid_iq.output_ramp, 4));
      return;
    }
    if (p[1] == 'L' || p[1] == 'l') {
      if (p[2] != '\0')
        pid_iq.out_limit = atof(&p[2]);
      printf("PID curr q|limit:%s\r\n", f2s(pid_iq.out_limit, 4));
      return;
    }
  }

  // 14. 电流 D 轴调谐: MDP / DP...
  if (p[0] == 'D' || p[0] == 'd') {
    if (p[1] == 'P' || p[1] == 'p') {
      if (p[2] != '\0')
        pid_id.Kp = atof(&p[2]);
      printf("PID curr d|P: %s\r\n", f2s(pid_id.Kp, 4));
      return;
    }
    if (p[1] == 'I' || p[1] == 'i') {
      if (p[2] != '\0')
        pid_id.Ki = atof(&p[2]);
      printf("PID curr d|I: %s\r\n", f2s(pid_id.Ki, 4));
      return;
    }
    if (p[1] == 'D' || p[1] == 'd') {
      if (p[2] != '\0')
        pid_id.Kd = atof(&p[2]);
      printf("PID curr d|D: %s\r\n", f2s(pid_id.Kd, 4));
      return;
    }
    if (p[1] == 'R' || p[1] == 'r') {
      if (p[2] != '\0')
        pid_id.output_ramp = atof(&p[2]);
      printf("PID curr d|ramp:%s\r\n", f2s(pid_id.output_ramp, 4));
      return;
    }
    if (p[1] == 'L' || p[1] == 'l') {
      if (p[2] != '\0')
        pid_id.out_limit = atof(&p[2]);
      printf("PID curr d|limit:%s\r\n", f2s(pid_id.out_limit, 4));
      return;
    }
  }

  // 15. 传感器与物理参数: MSE/SE, MSM/SM, MR/R
  if (p[0] == 'S' || p[0] == 's') {
    if (p[1] == 'E' || p[1] == 'e') {
      if (p[2] != '\0')
        motor.zero_offset = atof(&p[2]);
      printf("Sensor |el. offset:%s\r\n", f2s(motor.zero_offset, 4));
      return;
    }
    if (p[1] == 'M' || p[1] == 'm') {
      printf("Sensor |offset:0.0000\r\n");
      return;
    }
  }
  if (p[0] == 'R' || p[0] == 'r') {
    printf("R phase:21.2000\r\n");
    return;
  }
  if (p[0] == 'W' || p[0] == 'w') {
    if (p[1] == 'C' || p[1] == 'c') {
      printf("PWM Mod | center:1.0000\r\n");
      return;
    }
    if (p[1] == 'T' || p[1] == 't') {
      printf("PWM Mod | type:SVPWM\r\n");
      return;
    }
  }

  // 16. 通用帮助与未知指令回显 (避免静默无声让用户以为卡死)
  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
    printf("--- SimpleFOC Commander ---\r\n");
    printf("Commands: ME1(enable), ME0(disable), M<val>(target)\r\n");
    printf("MC0(torque), MC1(vel), MC2(angle)\r\n");
    printf("MMD100(monitor 100ms), MMD0(stop)\r\n");
    return;
  }

  // 未知指令提示并回显收到的内容
  printf("Echo: [%s]\r\n", cmd);
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
