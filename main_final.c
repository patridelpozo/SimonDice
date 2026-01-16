/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SECUENCIA_LEN 15
#define INICIAL_SECUENCIA_LEN 3
#define TM1637_CLK_PORT GPIOB
#define TM1637_CLK_PIN  GPIO_PIN_6
#define TM1637_DIO_PORT GPIOB
#define TM1637_DIO_PIN  GPIO_PIN_7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile uint8_t juego_iniciado = 0;   // 0: Selección de velocidad, 1: Jugando
volatile uint8_t confirmacion_btn = 0; // Flag para el botón PC13
volatile int8_t boton_presionado = -1; // -1: Ninguno, 0-3: Botones PA0-PA3
const uint8_t SEG_EASY[] = { 0x79, 0x77, 0x6D, 0x6E }; // "EASY"
const uint8_t SEG_MED[]  = { 0x37, 0x79, 0x5E, 0x00 }; // "nEd " (La M es difícil, usamos n)
const uint8_t SEG_HARD[] = { 0x76, 0x77, 0x50, 0x5E }; // "HArd"
const uint8_t SEG_GO[]   = { 0x3D, 0x5C, 0x00, 0x00 }; // Go
const uint8_t SEG_FAIL[] = { 0x71, 0x77, 0x06, 0x38 }; // FAIL
uint8_t secuencia[MAX_SECUENCIA_LEN];
uint8_t longitud_actual = INICIAL_SECUENCIA_LEN;
uint8_t indice_usuario = 0;
uint8_t esperando_usuario = 0;
uint32_t tiempo_secuencia = 500; // Velocidad ajustable por ADC
uint32_t puntuacion_total = 0;
uint32_t multiplicador_global = 1;
volatile uint32_t centesimas = 0;
volatile uint8_t cronometro_activo = 0;
// --- Tabla de números para el Display ---
const uint8_t segment_map[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void MostrarSecuencia(void);
void AnimacionError(void);
void AnimacionExito(void);
void TM1637_Display_Number(uint16_t num);
void TM1637_Display_Message(const uint8_t segments[]);
void TM1637_WriteByte(uint8_t data);
void TM1637_Start(void);
void TM1637_Stop(void);
void TM1637_SetBrightness(uint8_t brightness, uint8_t on);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  srand(TIM2->CNT);
  TM1637_SetBrightness(7, 1);
   TM1637_Display_Number(0); // Empezar en 0000
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // ==========================================================
	 	      // FASE 1: SELECCIÓN DE DIFICULTAD (Ajuste por ADC)
	 	      // ==========================================================
	 	      while (juego_iniciado == 0)
	 	      {
	 	        // 1. Leer ADC para ajustar velocidad (Pin PA4)
	 	        HAL_ADC_Start(&hadc1);
	 	        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
	 	            uint32_t raw = HAL_ADC_GetValue(&hadc1);
	 	            // Mapeo: 100ms (muy rápido) a 900ms (lento)
	 	           if (raw < 1365) {
	 	                       // DIFICULTAD BAJA (Lento)
	 	                       tiempo_secuencia = 800;
	 	                       TM1637_Display_Message(SEG_EASY);
	 	                       multiplicador_global = 1;
	 	                   }
	 	                   else if (raw < 2730) {
	 	                       // DIFICULTAD MEDIA
	 	                       tiempo_secuencia = 450;
	 	                       TM1637_Display_Message(SEG_MED);
	 	                       multiplicador_global = 2;
	 	                   }
	 	                   else {
	 	                       // DIFICULTAD ALTA (Rápido)
	 	                       tiempo_secuencia = 200;
	 	                       TM1637_Display_Message(SEG_HARD);
	 	                       multiplicador_global = 4;
	 	                   }
	 	            //tiempo_secuencia = (raw * 800 / 4095) + 100;
	 	        }
	 	        HAL_ADC_Stop(&hadc1);

	 	        // 2. Feedback visual: Todos los LEDs parpadean al ritmo del ADC (PD0-PD3)
	 	        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_SET);
	 	        HAL_Delay(tiempo_secuencia / 2);
	 	        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
	 	        HAL_Delay(tiempo_secuencia / 2);

	 	        // 3. Si la interrupción activó 'confirmacion_btn', salimos del ajuste
	 	        if (confirmacion_btn) {
	 	            confirmacion_btn = 0;
	 	            boton_presionado = -1;
	 	           TM1637_Display_Message(SEG_GO);
	 	          HAL_Delay(1000);
	 	          puntuacion_total = 0;

	 	            longitud_actual = INICIAL_SECUENCIA_LEN;
	 	           juego_iniciado = 1;
	 	            esperando_usuario = 0;


	 	        }
	 	      }
	 	      // ==========================================================
	 	      // FASE 2: EL JUEGO (SIMÓN DICE)
	 	      // ==========================================================

	 	      // Si la CPU debe mostrar la secuencia
	 	      if (!esperando_usuario)
	 	      {
	 	    	  	  	  TM1637_Display_Number(longitud_actual);
	 	    	          HAL_Delay(500);
	 	                  MostrarSecuencia(); // <--- Llamada a la función
	 	                  indice_usuario = 0;
	 	                  esperando_usuario = 1;
	 	                  centesimas = 0;
	 	                  cronometro_activo = 1;
	 	                  HAL_TIM_Base_Start_IT(&htim3);
	 	              }

	 	      if (esperando_usuario && boton_presionado != -1)
	 	      {
	 	          int8_t color_pulsado = boton_presionado;
	 	          boton_presionado = -1; // Reset del flag inmediatamente

	 	          // Feedback visual del botón que el usuario tocó
	 	          HAL_GPIO_WritePin(GPIOD, (GPIO_PIN_0 << color_pulsado), GPIO_PIN_SET);
	 	          HAL_Delay(250);
	 	          HAL_GPIO_WritePin(GPIOD, (GPIO_PIN_0 << color_pulsado), GPIO_PIN_RESET);

	 	          // Comprobar si acertó
	 	          if (color_pulsado == secuencia[indice_usuario])
	 	          {
	 	              indice_usuario++;

	 	              // ¿Ha completado toda la secuencia de esta ronda?
	 	              if (indice_usuario >= longitud_actual)
	 	              {
	 	            	 HAL_TIM_Base_Stop_IT(&htim3);
	 	            	 cronometro_activo = 0;
	 	            	 uint32_t puntos_base = 10 * longitud_actual; // Antes era 100
	 	            	 uint32_t bonus_tiempo = 0;
	 	                 if (centesimas < 1000) {
	 	            	     bonus_tiempo = (1000 - centesimas) / 20; // Antes era /2
	 	            	 }
          	              puntuacion_total += (puntos_base + bonus_tiempo) * multiplicador_global;
	 	            	  TM1637_Display_Number((uint16_t)puntuacion_total);
	 	                  AnimacionExito();      // <--- Usamos la función
	 	                  longitud_actual++;     // Siguiente nivel
	 	                  esperando_usuario = 0; // Vuelve a mostrar secuencia la CPU
	 	                  HAL_Delay(500);
	 	              }
	 	          }
	 	          else
	 	          {
	 	              // ERROR -> Animación de fallo y volver a Fase 1
	 	        	 HAL_TIM_Base_Stop_IT(&htim3);
	 	        	 cronometro_activo = 0;
	 	        	TM1637_Display_Message(SEG_FAIL);
	 	        	 AnimacionError();
	 	        	 HAL_Delay(1000);
	 	        	 TM1637_Display_Number((uint16_t)puntuacion_total);
	 	        	 HAL_Delay(2000);
	 	        	 juego_iniciado = 0;     // Regresa al menú de velocidad
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// --- LÓGICA TM1637 ---
void TM1637_Display_Number(uint16_t num) {
    uint8_t digits[4];
    digits[0] = segment_map[(num / 1000) % 10];
    digits[1] = segment_map[(num / 100) % 10];
    digits[2] = segment_map[(num / 10) % 10];
    digits[3] = segment_map[num % 10];

    TM1637_Start();
    TM1637_WriteByte(0xC0);
    for(int i = 0; i < 4; i++) TM1637_WriteByte(digits[i]);
    TM1637_Stop();
}
void TM1637_Display_Message(const uint8_t segments[]) {
    TM1637_Start();
    TM1637_WriteByte(0xC0);
    for(int i = 0; i < 4; i++) TM1637_WriteByte(segments[i]);
    TM1637_Stop();
}
void TM1637_Delay(void) { for(volatile int i = 0; i < 100; i++); }

void TM1637_Start(void) {
    HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_SET);
    TM1637_Delay();
    HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_RESET);
}

void TM1637_Stop(void) {
    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_RESET);
    TM1637_Delay();
    HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_RESET);
    TM1637_Delay();
    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_SET);
    TM1637_Delay();
    HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_SET);
}

void TM1637_WriteByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_RESET);
        TM1637_Delay();
        HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data >>= 1;
        TM1637_Delay();
        HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_SET);
        TM1637_Delay();
    }
    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_SET);
    TM1637_Delay();
    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_SET);
    TM1637_Delay();
}

void TM1637_SetBrightness(uint8_t brightness, uint8_t on) {
    uint8_t cmd = 0x80 | (on ? 0x08 : 0x00) | (brightness & 0x07);
    TM1637_Start();
    TM1637_WriteByte(cmd);
    TM1637_Stop();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        if (cronometro_activo) {
            centesimas++;
        }
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t ultimo_tick = 0;
  uint32_t ahora = HAL_GetTick();

  // Filtro anti-rebote (ignora pulsaciones más rápidas de 250ms)
  if (ahora - ultimo_tick < 250) return;
  ultimo_tick = ahora;

  // 1. Si estamos ajustando velocidad, el botón PC13 confirma el inicio
  if (GPIO_Pin == GPIO_PIN_13 && !juego_iniciado) {
      confirmacion_btn = 1;
  }

  // 2. Si el juego ya empezó y es el turno del jugador, leer botones PA0-PA3
  if (juego_iniciado && esperando_usuario) {
      if (GPIO_Pin == GPIO_PIN_0)      boton_presionado = 0;
      else if (GPIO_Pin == GPIO_PIN_1) boton_presionado = 1;
      else if (GPIO_Pin == GPIO_PIN_2) boton_presionado = 2;
      else if (GPIO_Pin == GPIO_PIN_3) boton_presionado = 3;
  }
}
void MostrarSecuencia(void) {
    for (int i = 0; i < longitud_actual; i++) {
        secuencia[i] = rand() % 4; // Genera color aleatorio 0-3
        HAL_GPIO_WritePin(GPIOD, (GPIO_PIN_0 << secuencia[i]), GPIO_PIN_SET);
        HAL_Delay(tiempo_secuencia);
        HAL_GPIO_WritePin(GPIOD, (GPIO_PIN_0 << secuencia[i]), GPIO_PIN_RESET);
        HAL_Delay(tiempo_secuencia / 2);
    }
}
void AnimacionExito(void) {

    for(int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOD, 0x0F, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOD, 0x0F, GPIO_PIN_RESET);
        HAL_Delay(100);
    }
}

void AnimacionError(void) {

    HAL_GPIO_WritePin(GPIOD, 0x0F, GPIO_PIN_SET);
    HAL_Delay(1500);
    HAL_GPIO_WritePin(GPIOD, 0x0F, GPIO_PIN_RESET);
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
