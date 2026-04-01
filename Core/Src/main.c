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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
extern float logf(float);  // 强制向编译器声明：不要管头文件里有没有，这个 FPU 单精度对数函数客观存在！
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// ADC 原始数据和计算后的温度值
uint32_t adc_raw_value = 0;
float current_temperature = 0.0f; 

// NTC 热敏电阻的关键参数 (MF58, 10k, B=3950)
// 注意：这些参数必须定义为 float，以充分利用 ARM 的 FPU 加速计算
const float R_REF = 10000.0f;       // 分压电阻 10kΩ
const float T0 = 298.15f;           // 标准温度 25℃ 的开尔文温度 (273.15 + 25)
const float R0 = 10000.0f;          // 标准温度 25℃ 时的 NTC 阻值 10kΩ
const float B_VALUE = 3950.0f;      // NTC 的 B 值

// PID 运算相关的临时变量 (后续会补全 PID 结构体)
float target_temperature = 25.0f; // 你的目标控制温度
float pwm_output = 0.0f;          // 计算出的 PWM 占空比输出值
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char display_buf[20]; // 用于存放转换后的温度字符串
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  将 ADC 原始值转换为摄氏温度 (利用 ARM FPU 硬件加速)
 * @param  adc_val: 16位过采样后的 ADC 原始值
 * @retval 计算出的摄氏温度 (float)
 */
float Calculate_Temperature(uint32_t adc_val) {
    // 1. 防止 ADC 读数为 0 导致除零错误或无穷大
    if (adc_val == 0 || adc_val >= 65535) { 
        return -99.0f; // 返回一个错误温度值
    }

    // 2. 计算当前 NTC 的阻值 Rt
    // 因为你开启了 16 位过采样，满量程是 65535。
    // 分压公式：V_ntc = (3.3V * Rt) / (R_ref + Rt) -> 转换为 ADC 值计算
    // Rt = (R_ref * adc_val) / (65535.0f - adc_val)
    float Rt = (R_REF * (float)adc_val) / (65535.0f - (float)adc_val);

    // 3. 使用 B 值公式计算开尔文温度 (Steinhart-Hart 简化方程)
    // 公式: T = 1 / ( (1/T0) + (1/B) * ln(Rt/R0) )
    // 注意：一定要用 logf() 单精度版本，不要用 log() 双精度版本，以发挥 FPU 最大性能！
    float T_Kelvin = 1.0f / ( (1.0f / T0) + (1.0f / B_VALUE) * logf(Rt / R0) );

    // 4. 将开尔文温度转换为摄氏温度
    float T_Celsius = T_Kelvin - 273.15f;

    return T_Celsius;
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // 启动 ADC 硬件自校准（极大降低零点漂移误差） - 绝对保留
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // 启动 TIM2 的通道 3 (PA2) 和 通道 4 (PA3) 的 PWM 输出 - 新增代码
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	OLED_Init();      // 初始化 OLED
  OLED_Clear();     // 清屏
  OLED_ShowString(0, 0, "TEC Control System", 12); // 显示标题
  OLED_ShowString(0, 2, "Target: 35.0 C", 12);     // 显示老师给的目标值
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1. 触发一次 ADC 转换
    HAL_ADC_Start(&hadc1);

    // 2. 等待转换完成 (带超时机制)
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        
        // 3. 读取 16 位过采样后的真实数据
        adc_raw_value = HAL_ADC_GetValue(&hadc1);
        
        // 4. 利用 ARM FPU 解算温度
        current_temperature = Calculate_Temperature(adc_raw_value);
    }
    
    // 5. 停止 ADC (为下一次触发做准备)
    HAL_ADC_Stop(&hadc1);

    // 稍微延时一下，防止循环过快，方便用 Keil 的 Watch 窗口观察
    HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  // 让 DRV8833 的 IN1 (PA2) 输出 50% 的 PWM 波 (250 / 500 = 50%)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 250);
  
  // 让 DRV8833 的 IN2 (PA3) 输出低电平 (0%)，形成压差
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
		
		// 将浮点数转换为字符串，保留一位小数
    sprintf(display_buf, "Temp: %.1f C  ", current_temperature);
    
    // 在屏幕第 4 行显示当前温度
    OLED_ShowString(0, 4, (uint8_t *)display_buf, 16);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
