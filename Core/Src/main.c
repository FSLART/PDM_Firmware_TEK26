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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "data_t26.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Snapshot de todo o estado útil para debug - ver como Live Expression
 (adiciona só "dbg" e expande). Atualizado a cada iteração do loop. */
typedef struct {
	/* PWM aplicado (0-100%) */
	uint8_t fan_pwm;
	uint8_t pump_pwm;
	uint8_t cooling_active;

	/* Temperaturas dos inversores (°C) */
	float inv1_inverter_temp_c;
	float inv1_motor_temp_c;
	float inv2_inverter_temp_c;
	float inv2_motor_temp_c;
	uint32_t temps_age_ms;      // há quanto tempo não chega frame de temperatura

	/* LV battery / corrente */
	float lv_voltage_v;
	uint16_t lv_voltage_raw;
	float current_v;
	uint16_t current_raw;

	/* Estado do bus CAN */
	HAL_CAN_StateTypeDef can_state;
	uint32_t can_error_code;
	uint32_t can_rx_count;
	uint32_t can_tx_ok_count;
	uint32_t can_tx_fail_count;
	uint32_t can_last_rx_id;
	uint32_t can_rx_age_ms;     // há quanto tempo não chega frame nenhum

	uint32_t uptime_ms;
} debug_view_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 10
#define VREF_MV     3300   // STM32 reference voltage in mV

/* --- Cooling control --- */
#define COOLING_TEMP_MIN_C     35.0f   // Below this -> 0% PWM
#define COOLING_TEMP_MAX_C     65.0f   // At or above this -> 100% PWM
#define COOLING_HYSTERESIS_C   5.0f    // Só desliga abaixo de (MIN - isto)
#define COOLING_TABLE_SIZE     2

/* --- Periodic CAN TX --- */
#define TIM3_TICK_MS           100     // TIM3 interrupt period (72MHz/1001/7201 ~ 100ms)
#define CAN_TX_PERIOD_MS       1000    // All periodic messages sent every 1 second
#define CAN_TX_PERIOD_TICKS    (CAN_TX_PERIOD_MS / TIM3_TICK_MS)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* CAN */
CAN_TxHeaderTypeDef TxHeader; //TX
uint8_t TxData[8];            //TX
uint32_t TxMailbox;           //TX
CAN_RxHeaderTypeDef RxHeader; //RX
uint8_t RxData[8];            //RX

/* Variáveis Globais ADC e Controlo */
volatile uint16_t adc_dma_raw[2] = { 0 };
volatile uint8_t adc_dma_ready = 0;

uint16_t voltage_buffer[BUFFER_SIZE] = { 0 };
uint32_t voltage_sum = 0;
uint8_t voltage_idx = 0;

uint16_t current_buffer[BUFFER_SIZE] = { 0 };
uint32_t current_sum = 0;
uint8_t current_idx = 0;

uint16_t raw_voltage = 0;
uint16_t raw_current = 0;

float voltage_v = 0.0f;   // PA1 em Volts  ex: 3.009
float current_v = 0.0f;   // PA2 em Volts  ex: 3.530
uint16_t current_frac = 0;

/* Contadores do Timer (Sincronização) */
uint16_t adc_counter = 0;
uint8_t data_trigger = 0;
uint16_t lv_counter = 0;
uint16_t heartbeat_counter = 0;

/* Estatísticas CAN (para debug_view_t) */
volatile uint32_t can_rx_count = 0;
volatile uint32_t can_tx_ok_count = 0;
volatile uint32_t can_tx_fail_count = 0;
volatile uint32_t can_last_rx_id = 0;
volatile uint32_t last_rx_tick_ms = 0;
volatile uint32_t last_temps_tick_ms = 0;

/* Snapshot único para Live Expressions - ver debug_view_t */
volatile debug_view_t dbg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void StartADC1(void);
void VoltageMessure(uint16_t raw_voltage);
void MeasureCurrent(uint16_t adc_value);
void SendData(void);
void HeartbeatTask(void);
void Cooling_Update(void);
void PDM_SendPeriodic(void);
void Debug_Update(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Tabela de arrefecimento: temperatura -> PWM (%). Interpolação linear entre pontos. */
typedef struct {
	float temp_c;
	uint8_t pump_pwm;
	uint8_t fan_pwm;
} cooling_point_t;

static const cooling_point_t cooling_table[COOLING_TABLE_SIZE] = {
/* temp   pump  fan  */
{ 35.0f, 0, 0 }, { 65.0f, 100, 100 }, // linear entre COOLING_TEMP_MIN_C e COOLING_TEMP_MAX_C
		};

/* Últimas temperaturas recebidas dos inversores (ºC) */
float inv1_temp_inverter_c = 0.0f;
float inv1_temp_motor_c = 0.0f;
float inv2_temp_inverter_c = 0.0f;
float inv2_temp_motor_c = 0.0f;
volatile uint8_t inv_temps_updated = 0;

/* PWM atualmente aplicado (também vai no CAN PDM_Cooling) */
uint8_t pump_pwm_now = 0;
uint8_t fan_pwm_now = 0;
uint8_t cooling_active = 0;   // estado da histerese (ver Cooling_Update)

uint16_t pdm_tx_counter = 0;   // incrementado pelo TIM3, dispara TX a 1s

/* EUSART */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

void Radiator_SetPWM(uint8_t percentagem)   // 0..100
{
	if (percentagem > 100)
		percentagem = 100;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t )percentagem * 10);
}

void WaterPump_SetPWM(uint8_t percentagem)  // 0..100
{
	if (percentagem > 100)
		percentagem = 100;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t )percentagem * 10);
}

/* CAN RX */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		return;

	/* Todos os IDs deste projeto são standard de 11 bits. Frames extended
	 são tráfego alheio - ignorar antes do switch (ver decode do VCU). */
	if (RxHeader.IDE == CAN_ID_EXT)
		return;

	can_rx_count++;
	can_last_rx_id = RxHeader.StdId;
	last_rx_tick_ms = HAL_GetTick();

	switch (RxHeader.StdId) {

	/* ---- Comando ON/OFF manual (0x23) ---- */
	case 0x23:
		if (RxData[5] == 0x00) {
			Radiator_SetPWM(0);
			WaterPump_SetPWM(0);
			HAL_GPIO_WritePin(GPIOA, WaterPump_Pin, GPIO_PIN_SET);
			printf("Radiator, AMS, WaterPump OFF\r\n");
		} else if (RxData[5] == 0x01) {
			Radiator_SetPWM(100);
			WaterPump_SetPWM(100);
			HAL_GPIO_WritePin(GPIOA, WaterPump_Pin, GPIO_PIN_RESET);
			printf("Radiator, AMS, WaterPump ON\r\n");
		}
		break;

		/* ---- Temperaturas INV1 (0x444) ---- */
	case DATA_T26_INV1_TEMPERATURES_FRAME_ID: {
		struct data_t26_inv1_temperatures_t m;
		if (data_t26_inv1_temperatures_unpack(&m, RxData, RxHeader.DLC) == 0) {
			inv1_temp_inverter_c = data_t26_inv1_temperatures_inv1_temp_inverter_decode(m.inv1_temp_inverter);
			inv1_temp_motor_c = data_t26_inv1_temperatures_inv1_temp_motor_decode(m.inv1_temp_motor);
			inv_temps_updated = 1;
			last_temps_tick_ms = HAL_GetTick();
		}
		break;
	}

		/* ---- Temperaturas INV2 (0x445) ---- */
	case DATA_T26_INV2_TEMPERATURES_FRAME_ID: {
		struct data_t26_inv2_temperatures_t m;
		if (data_t26_inv2_temperatures_unpack(&m, RxData, RxHeader.DLC) == 0) {
			inv2_temp_inverter_c = data_t26_inv2_temperatures_inv2_temp_inverter_decode(m.inv2_temp_inverter);
			inv2_temp_motor_c = data_t26_inv2_temperatures_inv2_temp_motor_decode(m.inv2_temp_motor);
			inv_temps_updated = 1;
			last_temps_tick_ms = HAL_GetTick();
		}
		break;
	}

	default:
		break;
	}
}

/* Callback do Timer 3 para incrementar os contadores sem bloquear o código */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		adc_counter++;
		data_trigger++;
		lv_counter++;
		heartbeat_counter++;
		pdm_tx_counter++;
	}
}

// Called automatically by HAL when DMA finishes transferring both channels
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		adc_dma_ready = 1;   // Signal the main loop
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_CAN_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	/* Iniciar Timer 3 com Interrupções para os Contadores */
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
		Error_Handler();
	}

	/* Iniciar geração de PWM (bomba de água e ventoinha) - sem isto o CCR
	   é atualizado mas nunca chega ao pino */
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}

	/* CAN */
	HAL_CAN_Start(&hcan);
	TxHeader.StdId = 0x123; // ID
	TxHeader.ExtId = 0x00;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 3;
	TxHeader.TransmitGlobalTime = DISABLE;

	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	// EUSART Initial Message
	char *test_msg = "USART OK\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*) test_msg, strlen(test_msg), 100);

	// Start IO OFF
	HAL_GPIO_WritePin(GPIOB, Radiator_Pin, GPIO_PIN_SET); // Invert Logic 1 -> 0
	HAL_GPIO_WritePin(GPIOB, AMS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, WaterPump_Pin, GPIO_PIN_SET);

	/* Inicializar o LED LV desligado */
	HAL_GPIO_WritePin(Led_LV_GPIO_Port, Led_LV_Pin, GPIO_PIN_RESET);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		StartADC1();                  // Executa a leitura do ADC periodicamente
		//VoltageMessure(raw_voltage);  // Avalia tensão e atualiza LED LV
		//MeasureCurrent(raw_current);  // Calcula e processa a corrente
		//SendData();                   // Envia dados UART e CAN
		HeartbeatTask();              // Pisca o LED Heartbeat de forma não bloqueante

		Cooling_Update();     // interpola tabela e aplica PWM
		PDM_SendPeriodic();   // envia 0x210 e 0x220 a cada 1s
		Debug_Update();       // atualiza snapshot "dbg" para Live Expressions

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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */
	// Configurar o DMA handle manualmente (CubeMX não gerou isto)
	hdma_adc1.Instance = DMA1_Channel1;  // ADC1 usa sempre DMA1 Ch1
	hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;   // Avança no array destino
	hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  // 16-bit ADC
	hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;  // 16-bit destino
	hdma_adc1.Init.Mode = DMA_NORMAL;
	hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;

	if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
		Error_Handler();
	}

	__HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 2;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	CAN_FilterTypeDef canfilter;

	/* Aceita todas as mensagens - a filtragem é feita por software no callback */
	canfilter.FilterBank = 0;
	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilter.FilterIdHigh = 0x0000;
	canfilter.FilterIdLow = 0x0000;
	canfilter.FilterMaskIdHigh = 0x0000;   // máscara 0 -> aceita tudo
	canfilter.FilterMaskIdLow = 0x0000;
	canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilter.FilterActivation = ENABLE;

	HAL_CAN_ConfigFilter(&hcan, &canfilter);

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 71;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1000;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 7200;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 71;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LED_Hearthbeat_Pin | Led_Debug_P14_Pin | Led_Debug_P15_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Led_LV_GPIO_Port, Led_LV_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(AMS_GPIO_Port, AMS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : LED_Hearthbeat_Pin Led_Debug_P14_Pin Led_Debug_P15_Pin */
	GPIO_InitStruct.Pin = LED_Hearthbeat_Pin | Led_Debug_P14_Pin | Led_Debug_P15_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : Led_LV_Pin AMS_Pin */
	GPIO_InitStruct.Pin = Led_LV_Pin | AMS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Devolve o maior valor de 4 floats */
static float MaxOf4(float a, float b, float c, float d) {
	float max = a;
	if (b > max)
		max = b;
	if (c > max)
		max = c;
	if (d > max)
		max = d;
	return max;
}

/* Interpola a tabela de arrefecimento para uma dada temperatura */
void Cooling_LookupPWM(float temp_c, uint8_t *pump_pwm, uint8_t *fan_pwm) {
	/* Abaixo do primeiro ponto */
	if (temp_c <= cooling_table[0].temp_c) {
		*pump_pwm = cooling_table[0].pump_pwm;
		*fan_pwm = cooling_table[0].fan_pwm;
		return;
	}
	/* Acima do último ponto -> 100% */
	if (temp_c >= cooling_table[COOLING_TABLE_SIZE - 1].temp_c) {
		*pump_pwm = cooling_table[COOLING_TABLE_SIZE - 1].pump_pwm;
		*fan_pwm = cooling_table[COOLING_TABLE_SIZE - 1].fan_pwm;
		return;
	}
	/* Encontra o segmento e interpola linearmente */
	for (uint8_t i = 0; i < COOLING_TABLE_SIZE - 1; i++) {
		if (temp_c < cooling_table[i + 1].temp_c) {
			float t0 = cooling_table[i].temp_c;
			float t1 = cooling_table[i + 1].temp_c;
			float frac = (temp_c - t0) / (t1 - t0);   // 0.0 .. 1.0 dentro do segmento

			*pump_pwm = (uint8_t) (cooling_table[i].pump_pwm + frac * (cooling_table[i + 1].pump_pwm - cooling_table[i].pump_pwm));
			*fan_pwm = (uint8_t) (cooling_table[i].fan_pwm + frac * (cooling_table[i + 1].fan_pwm - cooling_table[i].fan_pwm));
			return;
		}
	}
}

/* Corre no main loop: pega na temperatura mais alta e aplica o PWM */
void Cooling_Update(void) {
	if (!inv_temps_updated)
		return;
	inv_temps_updated = 0;

	float max_temp = MaxOf4(inv1_temp_inverter_c, inv1_temp_motor_c, inv2_temp_inverter_c, inv2_temp_motor_c);

	/* Histerese: liga ao atingir COOLING_TEMP_MIN_C, só desliga abaixo de
	 (COOLING_TEMP_MIN_C - COOLING_HYSTERESIS_C) - evita oscilar à volta do limiar */
	if (!cooling_active) {
		if (max_temp >= COOLING_TEMP_MIN_C)
			cooling_active = 1;
	} else {
		if (max_temp < (COOLING_TEMP_MIN_C - COOLING_HYSTERESIS_C))
			cooling_active = 0;
	}

	if (cooling_active) {
		Cooling_LookupPWM(max_temp, &pump_pwm_now, &fan_pwm_now);
	} else {
		pump_pwm_now = 0;
		fan_pwm_now = 0;
	}

	WaterPump_SetPWM(pump_pwm_now);
	Radiator_SetPWM(fan_pwm_now);
}

/* Helper genérico de TX (evita repetir o código dos mailboxes) */
static void CAN_Send(uint16_t id, uint8_t *data, uint8_t dlc) {
	TxHeader.StdId = id;
	TxHeader.DLC = dlc;
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0 && HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) == HAL_OK) {
		can_tx_ok_count++;
	} else {
		can_tx_fail_count++;
	}
}

/* Preenche o snapshot "dbg" - chamar a cada iteração do loop principal */
void Debug_Update(void) {
	dbg.fan_pwm = fan_pwm_now;
	dbg.pump_pwm = pump_pwm_now;
	dbg.cooling_active = cooling_active;

	dbg.inv1_inverter_temp_c = inv1_temp_inverter_c;
	dbg.inv1_motor_temp_c = inv1_temp_motor_c;
	dbg.inv2_inverter_temp_c = inv2_temp_inverter_c;
	dbg.inv2_motor_temp_c = inv2_temp_motor_c;
	dbg.temps_age_ms = HAL_GetTick() - last_temps_tick_ms;

	dbg.lv_voltage_v = voltage_v;
	dbg.lv_voltage_raw = raw_voltage;
	dbg.current_v = current_v;
	dbg.current_raw = raw_current;

	dbg.can_state = HAL_CAN_GetState(&hcan);
	dbg.can_error_code = HAL_CAN_GetError(&hcan);
	dbg.can_rx_count = can_rx_count;
	dbg.can_tx_ok_count = can_tx_ok_count;
	dbg.can_tx_fail_count = can_tx_fail_count;
	dbg.can_last_rx_id = can_last_rx_id;
	dbg.can_rx_age_ms = HAL_GetTick() - last_rx_tick_ms;

	dbg.uptime_ms = HAL_GetTick();
}

/* Envia todas as mensagens periódicas (1 segundo) */
void PDM_SendPeriodic(void) {
	if (pdm_tx_counter < CAN_TX_PERIOD_TICKS)
		return;
	pdm_tx_counter = 0;

	uint8_t buf[8];

	/* PDM_LV (0x210) - tensão LV */
	struct data_t26_pdm_lv_t lv_msg;
	lv_msg.lv_voltage_m_v = data_t26_pdm_lv_lv_voltage_m_v_encode(voltage_v); // V -> mV
	data_t26_pdm_lv_pack(buf, &lv_msg, sizeof(buf));
	CAN_Send(DATA_T26_PDM_LV_FRAME_ID, buf, DATA_T26_PDM_LV_LENGTH);

	/* PDM_Cooling (0x220) - PWM atual da bomba e ventoinha */
	struct data_t26_pdm_cooling_t cool_msg;
	cool_msg.water_pump_pwm = pump_pwm_now;
	cool_msg.cooling_fan_pwm = fan_pwm_now;
	data_t26_pdm_cooling_pack(buf, &cool_msg, sizeof(buf));
	CAN_Send(DATA_T26_PDM_COOLING_FRAME_ID, buf, DATA_T26_PDM_COOLING_LENGTH);
}

void StartADC1(void) {
	if (adc_counter >= 1) {
		adc_counter = 0;

		// Verifica se o ADC não está ocupado (bitmask correta para HAL F1)
		uint32_t state = HAL_ADC_GetState(&hadc1);
		if ((state & HAL_ADC_STATE_REG_BUSY) == 0) {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_dma_raw, 2);
		}
	}

	if (adc_dma_ready) {
		adc_dma_ready = 0;

		// Moving average Voltage
		voltage_sum -= voltage_buffer[voltage_idx];
		voltage_buffer[voltage_idx] = adc_dma_raw[0];
		voltage_sum += adc_dma_raw[0];
		voltage_idx = (voltage_idx + 1) % BUFFER_SIZE;
		raw_voltage = voltage_sum / BUFFER_SIZE;

		// Moving average PA2
		current_sum -= current_buffer[current_idx];
		current_buffer[current_idx] = adc_dma_raw[1];
		current_sum += adc_dma_raw[1];
		current_idx = (current_idx + 1) % BUFFER_SIZE;
		raw_current = current_sum / BUFFER_SIZE;

		VoltageMessure(raw_voltage);
		MeasureCurrent(raw_current);
	}
}

void VoltageMessure(uint16_t adc_voltage) {
	// Converte o valor raw do ADC para Volts
	voltage_v = (float) adc_voltage * 3.3f / 4095.0f;

	static uint8_t lv_state = 0;

	/* --- LÓGICA DE ESTADOS --- */
	if (voltage_v >= 3.25f) {
		// Tensão máxima suportada (ou perto do limite do ADC)
		lv_state = 2;
	} else if (voltage_v < 2.8f) {
		// Baixa tensão
		lv_state = 1;
	} else {
		// Tensão Normal OK (ex: 3.0V)
		lv_state = 0;
	}

	/* --- CONTROLO DO LED --- */
	switch (lv_state) {
	case 2: // TENSÃO MÁXIMA -> Estático LIGADO
		lv_counter = 0;
		HAL_GPIO_WritePin(Led_LV_GPIO_Port, Led_LV_Pin, GPIO_PIN_SET);
		break;

	case 1: // BAIXA TENSÃO -> Piscar
		// O valor '5' dita a velocidade do piscar com base no teu Timer 3.
		// Podes aumentar para piscar mais devagar, ou diminuir para piscar mais rápido.
		if (lv_counter >= 5) {
			lv_counter = 0;
			HAL_GPIO_TogglePin(Led_LV_GPIO_Port, Led_LV_Pin);
		}
		break;

	case 0: // TENSÃO NORMAL -> Estático DESLIGADO
		lv_counter = 0;
		HAL_GPIO_WritePin(Led_LV_GPIO_Port, Led_LV_Pin, GPIO_PIN_RESET);
		break;
	}
}

void MeasureCurrent(uint16_t adc_value) {
	current_v = (float) adc_value * 3.3f / 4095.0f;
}

void SendData(void) {
	if (data_trigger >= 10) {
		data_trigger = 0;

		// UART — imprime float diretamente
		printf("PA1: %.3f V | PA2: %.3f V\r\n", voltage_v, current_v);

		// CAN — envia como inteiro x1000 (ex: 3.009V → 3009)
		// Assim preservas 3 casas decimais sem float no CAN
		uint16_t can_voltage = (uint16_t) (voltage_v * 1000.0f);  // ex: 3009
		uint16_t can_current = (uint16_t) (current_v * 1000.0f);  // ex: 3530

		TxData[0] = (can_voltage >> 8) & 0xFF;  // High byte
		TxData[1] = can_voltage & 0xFF;  // Low byte
		TxData[2] = (can_current >> 8) & 0xFF;
		TxData[3] = can_current & 0xFF;
		TxHeader.DLC = 4;

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
			HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0);
			HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX1);
			HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX2);
			printf("WARNING: CAN congestionado.\r\n");
		}

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
			HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
			HAL_GPIO_TogglePin(GPIOC, Led_Debug_P15_Pin);
		} else {
			printf("CRITICAL ERROR: Falha CAN.\r\n");
		}
	}
}

void HeartbeatTask(void) {
	// Substitui o HAL_Delay de 5 segundos que trancava o Loop.
	// Pisca o LED da mesma forma, mas permitindo que a CAN e o ADC fluam
	if (heartbeat_counter >= 5) { // Ajuste este valor dependendo do tick rate do TIM3 para chegar a 5000ms ou 500ms
		heartbeat_counter = 0;
		HAL_GPIO_TogglePin(GPIOC, LED_Hearthbeat_Pin);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
