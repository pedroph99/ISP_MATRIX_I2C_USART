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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PCF8591_ADDR (0x48U << 1)
#define UART_LINE_MAX 32U
#define SPI_FRAME_MAX 13U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t uart_rx_byte;
static char uart_line[UART_LINE_MAX];
static uint8_t uart_line_len;
static volatile uint8_t uart_line_ready;

typedef enum { I2C_IDLE, I2C_READ_CONTROL, I2C_READ_DATA, I2C_WRITE_DAC } I2C_Operation;
static volatile I2C_Operation i2c_operation = I2C_IDLE;
static uint8_t i2c_tx[2];
static uint8_t i2c_rx[2];
static uint8_t i2c_channel;
static uint8_t dac_value;
static uint8_t dac_enabled;
static volatile uint8_t adc_ready;
static volatile uint8_t adc_value;
static volatile uint8_t adc_channel;
static volatile uint8_t dac_ready;
static volatile uint8_t i2c_error;

static uint8_t spi_frames[SPI_FRAME_MAX][2];
static uint8_t spi_count;
static uint8_t spi_pos;
static volatile uint8_t spi_active;
static volatile uint8_t spi_error;
static char wanted_glyph;
static char sent_glyph;
static char sensor_glyph;
static char sign_glyph;
static uint8_t show_sign;
static uint32_t next_switch_ms;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void App_Task(void);
static void App_Reply(const char *text);
static void App_ProcessCommand(const char *command);
static void App_StartRead(uint8_t channel);
static void App_StartDAC(uint8_t value);
static void Matrix_Init(void);
static void Matrix_Request(char glyph);
static void Matrix_Kick(void);
static void SPI_StartSequence(void);
static void SPI_StartFrame(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Padroes de 8 linhas. O bit 7 e o LED mais a esquerda. */
static const uint8_t glyph_T[8]     = {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00};
static const uint8_t glyph_V[8]     = {0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00};
static const uint8_t glyph_L[8]     = {0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00};
static const uint8_t glyph_plus[8]  = {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00};
static const uint8_t glyph_minus[8] = {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00};

static const uint8_t *Matrix_Pattern(char glyph)
{
  switch (glyph) {
    case 'T': return glyph_T;
    case 'V': return glyph_V;
    case 'L': return glyph_L;
    case '+': return glyph_plus;
    case '-': return glyph_minus;
    default:  return NULL;
  }
}

static void App_Reply(const char *text)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

static void SPI_StartFrame(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit_IT(&hspi1, spi_frames[spi_pos], 2) != HAL_OK) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    spi_active = 0;
    spi_error = 1;
  }
}

static void SPI_StartSequence(void)
{
  spi_pos = 0;
  spi_active = 1;
  SPI_StartFrame();
}

static void Matrix_Init(void)
{
  spi_count = 0;
  spi_frames[spi_count][0] = 0x0F; spi_frames[spi_count++][1] = 0x00; /* teste off */
  spi_frames[spi_count][0] = 0x09; spi_frames[spi_count++][1] = 0x00; /* sem decode */
  spi_frames[spi_count][0] = 0x0B; spi_frames[spi_count++][1] = 0x07; /* 8 linhas */
  spi_frames[spi_count][0] = 0x0A; spi_frames[spi_count++][1] = 0x08; /* brilho */
  for (uint8_t row = 0; row < 8; row++) {
    spi_frames[spi_count][0] = (uint8_t)(row + 1U);
    spi_frames[spi_count++][1] = 0x00;
  }
  spi_frames[spi_count][0] = 0x0C; spi_frames[spi_count++][1] = 0x01; /* liga */
  SPI_StartSequence();
}

static void Matrix_Request(char glyph)
{
  wanted_glyph = glyph;
}

static void Matrix_Kick(void)
{
  if (spi_active || wanted_glyph == 0 || wanted_glyph == sent_glyph) return;
  const uint8_t *pattern = Matrix_Pattern(wanted_glyph);
  if (pattern == NULL) return;

  spi_count = 8;
  for (uint8_t row = 0; row < 8; row++) {
    spi_frames[row][0] = (uint8_t)(row + 1U);
    spi_frames[row][1] = pattern[row];
  }
  SPI_StartSequence();
  if (spi_active) sent_glyph = wanted_glyph;
}

static void App_StartRead(uint8_t channel)
{
  if (i2c_operation != I2C_IDLE) {
    App_Reply("I2C ocupado\r\n");
    return;
  }
  i2c_channel = channel;
  /* Mantem o DAC ativo se Set_DAC ja foi usado. */
  i2c_tx[0] = (uint8_t)((dac_enabled ? 0x40U : 0x00U) | channel);
  i2c_operation = I2C_READ_CONTROL;
  if (HAL_I2C_Master_Transmit_IT(&hi2c1, PCF8591_ADDR, i2c_tx, 1) != HAL_OK) {
    i2c_operation = I2C_IDLE;
    App_Reply("Falha ao iniciar leitura I2C\r\n");
  }
}

static void App_StartDAC(uint8_t value)
{
  if (i2c_operation != I2C_IDLE) {
    App_Reply("I2C ocupado\r\n");
    return;
  }
  dac_value = value;
  i2c_tx[0] = 0x40; /* habilita saida analogica */
  i2c_tx[1] = value;
  i2c_operation = I2C_WRITE_DAC;
  if (HAL_I2C_Master_Transmit_IT(&hi2c1, PCF8591_ADDR, i2c_tx, 2) != HAL_OK) {
    i2c_operation = I2C_IDLE;
    App_Reply("Falha ao iniciar escrita I2C\r\n");
  }
}

static void App_ProcessCommand(const char *command)
{
  if (strcmp(command, "Read_AIN0") == 0 || strcmp(command, "LDR") == 0) {
    App_StartRead(0);
  } else if (strcmp(command, "Read_AIN1") == 0 || strcmp(command, "Temp") == 0) {
    App_StartRead(1);
  } else if (strcmp(command, "Read_AIN3") == 0 || strcmp(command, "Volt") == 0) {
    App_StartRead(3);
  } else if (strncmp(command, "Set_DAC_", 8) == 0) {
    char *end;
    long value = strtol(command + 8, &end, 10);
    if (end == command + 8 || *end != '\0' || value < 0 || value > 255) {
      App_Reply("Use Set_DAC_0 a Set_DAC_255\r\n");
    } else {
      App_StartDAC((uint8_t)value);
    }
  } else {
    App_Reply("Comandos: LDR, Temp, Volt, Read_AIN0/1/3, Set_DAC_0..255\r\n");
  }
}

static void App_Task(void)
{
  char command[UART_LINE_MAX];
  if (uart_line_ready) {
    __disable_irq();
    strcpy(command, uart_line);
    uart_line_len = 0;
    uart_line_ready = 0;
    __enable_irq();
    App_ProcessCommand(command);
  }

  if (adc_ready) {
    uint8_t channel, value;
    __disable_irq();
    channel = adc_channel;
    value = adc_value;
    adc_ready = 0;
    __enable_irq();

    sensor_glyph = (channel == 0) ? 'L' : (channel == 1) ? 'T' : 'V';
    sign_glyph = (value < 128U) ? '-' : '+';
    show_sign = 0;
    next_switch_ms = HAL_GetTick() + 500U;
    Matrix_Request(sensor_glyph);

    char response[48];
    snprintf(response, sizeof(response), "AIN%u = %u / 255\r\n",
             (unsigned)channel, (unsigned)value);
    App_Reply(response);
  }

  if (dac_ready) {
    dac_ready = 0;
    char response[48];
    snprintf(response, sizeof(response), "DAC = %u / 255\r\n",
             (unsigned)dac_value);
    App_Reply(response);
  }
  if (i2c_error) {
    i2c_error = 0;
    App_Reply("Erro I2C: confira endereco, SDA, SCL e GND\r\n");
  }
  if (spi_error) {
    spi_error = 0;
    App_Reply("Erro SPI\r\n");
  }

  if (sensor_glyph != 0 &&
      (int32_t)(HAL_GetTick() - next_switch_ms) >= 0) {
    next_switch_ms += 500U;
    show_sign ^= 1U;
    Matrix_Request(show_sign ? sign_glyph : sensor_glyph);
  }
  Matrix_Kick();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2) return;
  if (!uart_line_ready) {
    if (uart_rx_byte == '\r' || uart_rx_byte == '\n') {
      if (uart_line_len > 0) {
        uart_line[uart_line_len] = '\0';
        uart_line_ready = 1;
      }
    } else if (uart_line_len < UART_LINE_MAX - 1U) {
      uart_line[uart_line_len++] = (char)uart_rx_byte;
    } else {
      uart_line_len = 0; /* descarta linha longa */
    }
  }
  HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
  }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1) return;
  if (i2c_operation == I2C_READ_CONTROL) {
    i2c_operation = I2C_READ_DATA;
    /* O primeiro byte e da conversao anterior; o segundo e o atual. */
    if (HAL_I2C_Master_Receive_IT(&hi2c1, PCF8591_ADDR, i2c_rx, 2) != HAL_OK) {
      i2c_operation = I2C_IDLE;
      i2c_error = 1;
    }
  } else if (i2c_operation == I2C_WRITE_DAC) {
    dac_enabled = 1;
    i2c_operation = I2C_IDLE;
    dac_ready = 1;
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1 || i2c_operation != I2C_READ_DATA) return;
  adc_channel = i2c_channel;
  adc_value = i2c_rx[1];
  i2c_operation = I2C_IDLE;
  adc_ready = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) {
    i2c_operation = I2C_IDLE;
    i2c_error = 1;
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance != SPI1) return;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
  spi_pos++;
  if (spi_pos < spi_count) {
    SPI_StartFrame();
  } else {
    spi_active = 0;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    spi_active = 0;
    spi_error = 1;
  }
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
/* Inicia recepcao do terminal e configura a matriz por SPI com interrupcao. */
if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1) != HAL_OK) Error_Handler();
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
Matrix_Init();
App_Reply("Pronto: LDR, Temp, Volt, Read_AIN0/1/3, Set_DAC_0..255\r\n");
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Task();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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


