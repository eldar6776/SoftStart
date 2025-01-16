/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum
{
    INIT = 0,
    IDLE,
    START,
    RUN,
    STOP,
    SYS_ERROR
} sys_state_td;


#define ADC_READOUT_PERIOD 100
#define NTC_RREF                    10000U  // 10k NTC value of at 25 degrees
#define NTC_B_VALUE                 3977U   // NTC beta parameter
#define NTC_PULLUP                  10000U	// 10k pullup resistor
#define STARTUP_TIME                2000U   // max startup time
#define MIN_ANGLE                   80      // shutdown triac before zeroross
#define START_ANGLE                 75      // zerocros x 100uS triac startup angle
#define STOP_ANGLE                  5       // zerocros x 100uS triac stop angle
#define RELAY_SW_TIME               20      // 20mS relay to switch state
#define TRAFO_TEMP_TRESHOLD         400     // adc value for trafo ntc sensor overtemperature  error
#define TRIAC_TEMP_TRESHOLD         400     // adc value for triac ntc sensor overtemperature  error
#define SENSOR_VALID_DN_TRESHOLD    10      // adc value below this means that the sensor is not used
#define SENSOR_VALID_UP_TRESHOLD    4000    // adc value above this means that the sensor is open
#define SS_START                    0xA1
#define SS_STOP                     0xA2
#define SS_START_ALL                0xA3
#define SS_STOP_ALL                 0xA4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SoftStartReady()                (sys_flags |= (1U << 0))
#define SoftStartError()                (sys_flags &= (~(1U << 0)))
#define SoftStartOn()                   (sys_flags |= (1U << 1))
#define SoftStartOff()                  (sys_flags &= (~(1U << 1)))
#define TrafoTemperatureSensorUsed()    (sys_flags |= (1U << 2))
#define TrafoTemperatureSensorNotUsed() (sys_flags &= (~(1U << 2)))
#define IsTrafoTemperatureSensorUsed()  (sys_flags & (1U << 2))
#define TriacTemperatureSensorUsed()    (sys_flags |= (1U << 3))
#define TriacTemperatureSensorNotUsed() (sys_flags &= (~(1U << 3)))
#define IsTriacTemperatureSensorUsed()  (sys_flags & (1U << 3))
#define TrafoTemperatureErrorSet()      (sys_flags |= (1U << 4))
#define TrafoTemperatureErrorReset()    (sys_flags &= (~(1U << 4)))
#define IsTrafoTemperatureErrorActiv()  (sys_flags & (1U << 4))
#define TriacTemperatureErrorSet()      (sys_flags |= (1U << 5))
#define TriacTemperatureErrorReset()    (sys_flags &= (~(1U << 5)))
#define IsTriacTemperatureErrorActiv()  (sys_flags & (1U << 5))
#define TimeoutErrorSet()               (sys_flags |= (1U << 6))
#define TimeoutErrorReset()             (sys_flags &= (~(1U << 6)))
#define IsTimeoutErrorActiv()           (sys_flags & (1U << 6))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CRC_HandleTypeDef hcrc;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
TinyFrame tfapp;
uint8_t rec, addr;
uint8_t sys_flags;
uint8_t led_cnt;
uint8_t led_rep;
uint8_t error_cnt;

uint16_t trafo_temp;
uint16_t triac_temp;
volatile uint16_t tmr;
volatile uint8_t blink_done = 0;
uint32_t chktmr;
uint32_t start_tmr;
uint32_t start_tout;
uint32_t led_tmr;
uint32_t led_tout;
uint32_t start_led_tmr;

bool init_tf = false;
volatile bool start = false;
volatile bool stop = false;
volatile bool end = false;
volatile bool error = false;
volatile bool remote = false;
static bool trigger1 = false;
static bool trigger2 = false;
sys_state_td sys_state = INIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void RS485_Tick(void);
void RS485_Init(void);
static void ADC3_Read(void);
static void Error_Signal(void);
static uint8_t ReadAddressAndDelay(void);
static void HandleLedBlink(uint32_t repetitions);
TF_Result STATUS_Listener(TinyFrame *tf, TF_Msg *msg);
TF_Result SOFTSTART_Listener(TinyFrame *tf, TF_Msg *msg);
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
    MX_ADC_Init();
    MX_IWDG_Init();
    MX_TIM14_Init();
    MX_USART1_UART_Init();
    MX_CRC_Init();
    /* USER CODE BEGIN 2 */
    RS485_Init();


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while(1)
    {
        ADC3_Read();
        if(error == true) sys_state = SYS_ERROR;
        switch(sys_state)
        {
        case INIT:
            HAL_Delay(500);
            HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(STATUS_OUT_GPIO_Port, STATUS_OUT_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RELAY_CTRL_GPIO_Port, RELAY_CTRL_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(TRIAC_CTRL_GPIO_Port, TRIAC_CTRL_Pin, GPIO_PIN_RESET);
            addr = ReadAddressAndDelay();
            start_tout = addr * 2000U;
            HAL_Delay(500);
            SoftStartReady();
            TimeoutErrorReset();
            TriacTemperatureErrorReset();
            TrafoTemperatureErrorReset();
            sys_state = IDLE;
            trigger1 = false;
            trigger2 = false;
            remote = false;
            start = false;
            stop = false;
            end  = false;
            led_cnt = 0;
            led_rep = 0;
            led_tmr = 0;
            error_cnt = 0;
            led_tout = 0;
            break;

        case IDLE:
            SoftStartOff();
            SoftStartReady();
            if(!start_tmr && (remote || (HAL_GPIO_ReadPin(CTRL_IN_GPIO_Port, CTRL_IN_Pin) == GPIO_PIN_SET)))
            {
                start_tmr = HAL_GetTick();
                start_led_tmr = HAL_GetTick();
            }
            else if(start_tmr && !remote && (HAL_GPIO_ReadPin(CTRL_IN_GPIO_Port, CTRL_IN_Pin) == GPIO_PIN_RESET))
            {
                start_tmr = 0;
                start_led_tmr = 0;
                HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
            }
            if(start_tmr && ((HAL_GetTick() - start_tmr) >= start_tout))
            {
                HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(STATUS_OUT_GPIO_Port, STATUS_OUT_Pin, GPIO_PIN_SET);
                chktmr = HAL_GetTick();
                start_tmr = 0;
                start_led_tmr = 0;
                sys_state = START;
                tmr = START_ANGLE;
                trigger1 = false;
                trigger2 = false;
                start = true;
                stop = false;
                end  = false;
            }
            else if(start_led_tmr && ((HAL_GetTick() - start_led_tmr) >= 50))
            {
                start_led_tmr = HAL_GetTick();
                HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
            }
            break;

        case START:
            SoftStartOn();
            SoftStartReady();
            if(!remote && (HAL_GPIO_ReadPin(CTRL_IN_GPIO_Port, CTRL_IN_Pin) == GPIO_PIN_RESET))
            {
                chktmr = HAL_GetTick();
                sys_state = STOP;
                start = false;
                stop = true;
                end  = false;
            }
            else if(end == true)
            {
                HAL_GPIO_WritePin(RELAY_CTRL_GPIO_Port, RELAY_CTRL_Pin, GPIO_PIN_SET);
                chktmr = HAL_GetTick();
                sys_state = RUN;
            }
            if((HAL_GetTick() - chktmr) >= STARTUP_TIME)
            {
                TimeoutErrorSet();
                sys_state = SYS_ERROR;
            }
            break;

        case RUN:
            SoftStartOn();
            SoftStartReady();
            if(!remote && (HAL_GPIO_ReadPin(CTRL_IN_GPIO_Port, CTRL_IN_Pin) == GPIO_PIN_RESET))
            {
                chktmr = HAL_GetTick();
                tmr = STOP_ANGLE;
                sys_state = STOP;
                start = false;
                stop = true;
                end  = false;
                HAL_Delay(20);
                HAL_GPIO_WritePin(RELAY_CTRL_GPIO_Port, RELAY_CTRL_Pin, GPIO_PIN_RESET);
            }
            if((start && end) && ((HAL_GetTick() - chktmr) >= RELAY_SW_TIME))
            {
                start = false;
                stop = false;
                end  = false;
            }
            break;

        case STOP:
            SoftStartOff();
            SoftStartReady();
            if(end == true)
            {
                HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(STATUS_OUT_GPIO_Port, STATUS_OUT_Pin, GPIO_PIN_RESET);
                addr = ReadAddressAndDelay();
                start_tout = addr * 2000U;
                chktmr = HAL_GetTick();
                sys_state = IDLE;
                remote = false;
                start = false;
                stop = false;
                end  = false;
            }
            if((HAL_GetTick() - chktmr) >= STARTUP_TIME)
            {
                TimeoutErrorSet();
                sys_state = SYS_ERROR;
            }
            break;

        case SYS_ERROR:
            end = false;
            stop = false;
            start = false;
            SoftStartOff();
            SoftStartError();
            HAL_GPIO_WritePin(RELAY_CTRL_GPIO_Port, RELAY_CTRL_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(TRIAC_CTRL_GPIO_Port, TRIAC_CTRL_Pin, GPIO_PIN_RESET);
            if(!remote && (HAL_GPIO_ReadPin(CTRL_IN_GPIO_Port, CTRL_IN_Pin) == GPIO_PIN_RESET) &&
                    !IsTrafoTemperatureErrorActiv() &&
                    !IsTriacTemperatureErrorActiv())
            {
                error = false;
                sys_state = INIT;
            }
            Error_Signal();
            break;
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

#ifdef USE_WATCHDOG
        HAL_IWDG_Refresh(&hiwdg);
#endif
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
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                                       |RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI14CalibrationValue = 16;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

    /* USER CODE BEGIN ADC_Init 0 */

    /* USER CODE END ADC_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC_Init 1 */

    /* USER CODE END ADC_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_4;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC_Init 2 */

    /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance = CRC;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

    /* USER CODE BEGIN IWDG_Init 0 */
#ifdef USE_WATCHDOG
    /* USER CODE END IWDG_Init 0 */

    /* USER CODE BEGIN IWDG_Init 1 */

    /* USER CODE END IWDG_Init 1 */
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN IWDG_Init 2 */
#endif
    /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

    /* USER CODE BEGIN TIM14_Init 0 */

    /* USER CODE END TIM14_Init 0 */

    /* USER CODE BEGIN TIM14_Init 1 */

    /* USER CODE END TIM14_Init 1 */
    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 0;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 4799;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM14_Init 2 */

    /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

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
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, STATUS_LED_Pin|RELAY_CTRL_Pin|TRIAC_CTRL_Pin|STATUS_OUT_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : ADDRESS_0_Pin ADDRESS_1_Pin */
    GPIO_InitStruct.Pin = ADDRESS_0_Pin|ADDRESS_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pins : STATUS_LED_Pin RELAY_CTRL_Pin TRIAC_CTRL_Pin STATUS_OUT_Pin */
    GPIO_InitStruct.Pin = STATUS_LED_Pin|RELAY_CTRL_Pin|TRIAC_CTRL_Pin|STATUS_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : ZEROCROSS_Pin */
    GPIO_InitStruct.Pin = ZEROCROSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(ZEROCROSS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : CTRL_IN_Pin */
    GPIO_InitStruct.Pin = CTRL_IN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(CTRL_IN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : ADDRESS_2_Pin */
    GPIO_InitStruct.Pin = ADDRESS_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADDRESS_2_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static uint8_t ReadAddressAndDelay(void)
{
    uint8_t tmp = 0;
    if(HAL_GPIO_ReadPin(ADDRESS_0_GPIO_Port, ADDRESS_0_Pin) == GPIO_PIN_RESET) tmp |= 0x01U;
    if(HAL_GPIO_ReadPin(ADDRESS_1_GPIO_Port, ADDRESS_1_Pin) == GPIO_PIN_RESET) tmp |= 0x02U;
    if(HAL_GPIO_ReadPin(ADDRESS_2_GPIO_Port, ADDRESS_2_Pin) == GPIO_PIN_RESET) tmp |= 0x04U;
    return tmp;
}
/**
  * @brief  stop timer interrupt, fire triac, and compare current angle in 100us increments
  * @param
  * @retval
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    static uint32_t loctmr = 0;

    __HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);

    ++loctmr;
    if((loctmr >= tmr) && !trigger1)
    {
        trigger1 = true;
        trigger2 = false;
//		HAL_TIM_Base_Stop_IT(&htim14);
        HAL_GPIO_WritePin(TRIAC_CTRL_GPIO_Port, TRIAC_CTRL_Pin, GPIO_PIN_SET);
        if(stop == true)
        {
            if(tmr < START_ANGLE) ++tmr;
            else end = true;
        }
        else if(start == true)
        {
            if(tmr > STOP_ANGLE) --tmr;
            else end = true;
        }
    }
    else if((loctmr >= MIN_ANGLE) && !trigger2) {
        loctmr = 0;
        trigger1 = false;
        trigger2 = true;
        HAL_TIM_Base_Stop_IT(&htim14);
        HAL_GPIO_WritePin(TRIAC_CTRL_GPIO_Port, TRIAC_CTRL_Pin, GPIO_PIN_RESET);
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ZEROCROSS_Pin)
    {
        if(end == false) HAL_GPIO_WritePin(TRIAC_CTRL_GPIO_Port, TRIAC_CTRL_Pin, GPIO_PIN_RESET);
        if(((start == true) || (stop == true)) && (end == false))
        {
            HAL_TIM_Base_Stop_IT(&htim14);
            __HAL_TIM_SET_COUNTER(&htim14, 0);
            HAL_TIM_Base_Start_IT(&htim14);
        }
    }
}
/**
  * @brief
  * @param
  * @retval
  */
TF_Result STATUS_Listener(TinyFrame *tf, TF_Msg *msg)
{
    uint8_t ret[6];
    if(addr == msg->data[0])
    {
        ret[0] = sys_flags;
        ret[1] = sys_state;
        ret[2] = (trafo_temp >> 8);
        ret[3] = (trafo_temp & 0xFF);
        ret[4] = (triac_temp >> 8);
        ret[5] = (triac_temp & 0xFF);
        msg->data = ret;
        msg->len = 6;
        TF_Respond(tf, msg);
    }
    return TF_STAY;
}
/**
  * @brief
  * @param
  * @retval
  */
TF_Result SOFTSTART_Listener(TinyFrame *tf, TF_Msg *msg)
{
    uint8_t ret[6], res = 0;

    if(addr == msg->data[0])
    {
        switch(msg->data[1])
        {
        case SS_START:
            ++res;
        case SS_START_ALL:
            remote = true;
            break;
        case SS_STOP:
            ++res;
        case SS_STOP_ALL:
            remote = false;
            break;
        default:
            break;
        }
    }

    if(res)
    {
        ret[0] = sys_flags;
        ret[1] = sys_state;
        ret[2] = (trafo_temp >> 8);
        ret[3] = (trafo_temp & 0xFF);
        ret[4] = (triac_temp >> 8);
        ret[5] = (triac_temp & 0xFF);
        msg->data = ret;
        msg->len = 8;
        TF_Respond(tf, msg);
    }
    return TF_STAY;
}
/**
  * @brief
  * @param
  * @retval
  */
void RS485_Init(void)
{
    if(!init_tf)
    {
        init_tf = TF_InitStatic(&tfapp, TF_SLAVE);
        TF_AddTypeListener(&tfapp, V_SOFT_START, STATUS_Listener);
        TF_AddTypeListener(&tfapp, S_SOFT_START, SOFTSTART_Listener);
    }
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void RS485_Tick(void)
{
    if(init_tf == true)
    {
        TF_Tick(&tfapp);
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, len, 100);
    while(huart1.gState != HAL_UART_STATE_READY) continue;
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    TF_AcceptChar(&tfapp, rec);
    if(HAL_UART_Receive_IT(&huart1, &rec, 1) != HAL_OK)
    {
        Error_Handler();
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */

static void ADC3_Read(void)
{
    ADC_ChannelConfTypeDef sConfig;

    static uint8_t adc_cnt = 0U;
    static uint32_t adc_timer = 0U;


    if((HAL_GetTick() - adc_timer) >= ADC_READOUT_PERIOD)
    {
        adc_timer = HAL_GetTick();
        sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
        sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;

        if(adc_cnt == 0U)
        {
            sConfig.Channel = ADC_CHANNEL_0;
            HAL_ADC_ConfigChannel(&hadc, &sConfig);
            sConfig.Channel = ADC_CHANNEL_5;
            sConfig.Rank = ADC_RANK_NONE;
            HAL_ADC_ConfigChannel(&hadc, &sConfig);
        }
        else if(adc_cnt == 1U)
        {
            sConfig.Channel = ADC_CHANNEL_4;
            HAL_ADC_ConfigChannel(&hadc, &sConfig);
            sConfig.Channel = ADC_CHANNEL_0;
            sConfig.Rank = ADC_RANK_NONE;
            HAL_ADC_ConfigChannel(&hadc, &sConfig);
        }

        HAL_ADC_Start(&hadc);
        HAL_ADC_PollForConversion(&hadc, 10);

        if(adc_cnt == 0U)
        {
            triac_temp = HAL_ADC_GetValue(&hadc);
            if(triac_temp < SENSOR_VALID_DN_TRESHOLD)
            {
                TriacTemperatureSensorNotUsed();
                TriacTemperatureErrorReset();
            }
            else if((triac_temp <= TRIAC_TEMP_TRESHOLD) || (triac_temp > SENSOR_VALID_UP_TRESHOLD))
            {
                TriacTemperatureSensorUsed();
                TriacTemperatureErrorSet();
                error = true;
            }
            else
            {
                TriacTemperatureSensorUsed();
                TriacTemperatureErrorReset();
            }
            ++adc_cnt;
        }
        else if(adc_cnt == 1U)
        {
            trafo_temp = HAL_ADC_GetValue(&hadc);
            if(trafo_temp < SENSOR_VALID_DN_TRESHOLD)
            {
                TrafoTemperatureSensorNotUsed();
                TrafoTemperatureErrorReset();
            }
            else if((trafo_temp <= TRAFO_TEMP_TRESHOLD) || (trafo_temp > SENSOR_VALID_UP_TRESHOLD))
            {
                TrafoTemperatureSensorUsed();
                TrafoTemperatureErrorSet();
                error = true;
            }
            else
            {
                TrafoTemperatureSensorUsed();
                TrafoTemperatureErrorReset();
            }
            adc_cnt = 0;
        }
    }
}
/**
  * @brief
  * @param
  * @retval
  */
static void HandleLedBlink(uint32_t repetitions) {
    static uint32_t inner_counter = 0;

    if (inner_counter == 0) {
        ++led_cnt;
        led_rep = repetitions;
        led_tout = 200;
        led_tmr = HAL_GetTick();
        HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
        inner_counter++;
    } else {
        if (led_rep) {
            --led_rep;
            led_tmr = HAL_GetTick();
            HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
        } else {
            led_cnt = 0;
            led_rep = 0;
            led_tmr = 0;
            led_tout = 1000;
            HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
            inner_counter = 0;
            blink_done = 1; // Postavljanje blink_done na 1 kada je treptanje zavr�eno
        }
    }
}
/**
  * @brief
  * @param
  * @retval
  */
static void Error_Signal(void) {
    static uint8_t error_index = 0;
    static uint8_t error_order[] = {0, 1, 2};
    static uint8_t error_size = sizeof(error_order) / sizeof(error_order[0]);

    if ((HAL_GetTick() - led_tmr) >= led_tout) {
        switch (error_order[error_index]) {
        case 0:
            if (IsTrafoTemperatureErrorActiv() && blink_done == 0) {
                HandleLedBlink(0);
            } else {
                error_index++;
                blink_done = 0; // Resetovanje blink_done za sledeci slucaj gre�ke
            }
            break;
        case 1:
            if (IsTriacTemperatureErrorActiv() && blink_done == 0) {
                HandleLedBlink(2);
            } else {
                error_index++;
                blink_done = 0; // Resetovanje blink_done za sledeci slucaj gre�ke
            }
            break;
        case 2:
            if (IsTimeoutErrorActiv() && blink_done == 0) {
                HandleLedBlink(4);
            } else {
                error_index++;
                blink_done = 0; // Resetovanje blink_done za sledeci slucaj gre�ke
            }
            break;
        }

        if (error_index >= error_size) {
            error_index = 0; // Resetovanje na prvi slucaj gre�ke
        }

        if (led_cnt == 0) {
            led_tmr = HAL_GetTick();
        }
    }
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
    HAL_ADC_MspDeInit(&hadc);
    HAL_UART_MspDeInit(&huart1);
    HAL_TIM_Base_MspDeInit(&htim14);
    HAL_DeInit();

    while(1)
    {
        HAL_NVIC_SystemReset();
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
