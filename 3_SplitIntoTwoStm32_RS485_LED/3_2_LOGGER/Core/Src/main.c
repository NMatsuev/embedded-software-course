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
#include "fatfs.h"
#include "ds3231.h"
#include "Sd_spi.h"
#include "ssd1306.h" 
#include <stdio.h>
#include <string.h>
#include "link_proto.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NODE_ADDR         LINK_ADDR_LOGGER
#define LINK_RX_SIZE      256U       
#define LOG_EVERY_N       30U         
#define LINK_TIMEOUT_MS   7000U 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define MOCK_RX_ENABLE  0   /* 1 = inject fake frames for bench tests, never for real operation */
#define MOCK_PERIOD_MS  2000U

#if MOCK_RX_ENABLE
static const char *mock_frames[] =
{
    "$DHT,2,1,1799,236,507*74",
    "$DHT,2,1,1800,235,508*77",
    "$DHT,2,1,1801,235,507*79",
    "$DHT,2,1,1802,236,508*76",
    "$DHT,2,1,1803,235,508*74",
    "$DHT,2,1,1804,236,507*7F",
    "$DHT,2,1,1805,236,507*7E",
    "$DHT,2,1,1806,236,507*7D",
    "$DHT,2,1,1807,235,507*7F",
    "$DHT,2,1,1808,236,507*73",
    "$DHT,2,1,1809,235,507*71",
    "$DHT,2,1,1810,235,507*79",
    "$DHT,2,1,1811,236,508*74",
    "$DHT,2,1,1812,236,507*78",
    "$DHT,2,1,1813,235,507*7A",
    "$DHT,2,1,1814,235,507*7D",
    "$DHT,2,1,1815,235,508*73"
};

#define MOCK_FRAME_COUNT \
    (sizeof(mock_frames) / sizeof(mock_frames[0]))

static uint32_t mock_last_ms = 0;
static uint8_t mock_index = 0;
static uint8_t mock_started = 0;
#endif
		
extern volatile uint8_t sd_mounted;
static char csv_line[128];
static uint8_t dummy[512];  
extern volatile uint8_t sd_use_dma;
static uint8_t oled_ok = 0;

static volatile uint8_t  rx_ring[LINK_RX_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0;
static volatile uint32_t rx_overflow = 0;
volatile uint32_t uart_err = 0;          /* FE/NE/ORE counted in USART1_IRQHandler */
 
static link_rx_t   link_rx;
static uint8_t     link_seen = 0, link_ok = 0;
static uint32_t    last_rx_ms = 0;
static uint16_t    last_seq = 0;
static uint8_t     log_cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Print(char *message)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

static uint32_t sd_bench(uint8_t use_dma, uint8_t *buf, uint32_t n)
{
    uint32_t total_us = 0;
    sd_use_dma = use_dma;

    for (uint32_t i = 0; i < n; i++)
    {
        uint16_t t0 = __HAL_TIM_GET_COUNTER(&htim2);
        if (sd_read_block(buf, 0) != 0)
            return 0;                                    
        total_us += (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) - t0);
    }
    return total_us / n;                                
}

static void sd_dma_test(void)
{
    static uint8_t buf_poll[512], buf_dma[512];
    char s[96];

    uint32_t us_poll = sd_bench(0, buf_poll, 20);
    uint32_t us_dma  = sd_bench(1, buf_dma, 20);
    sd_use_dma = 1;                                     

    if (us_poll == 0 || us_dma == 0)
    {
        Print("[TEST] read error\r\n");
        return;
    }

    snprintf(s, sizeof(s), "[TEST] 512 B, no DMA: %lu us\r\n", (unsigned long)us_poll);
    Print(s);
    snprintf(s, sizeof(s), "[TEST] 512 B, DMA   : %lu us\r\n", (unsigned long)us_dma);
    Print(s);
    snprintf(s, sizeof(s), "[TEST] speedup x%lu.%lu, data %s\r\n",
             (unsigned long)(us_poll * 10 / us_dma / 10),
             (unsigned long)(us_poll * 10 / us_dma % 10),
             memcmp(buf_poll, buf_dma, 512) == 0 ? "identical" : "DIFFERENT");
    Print(s);
}

static void oled_show_humidity(uint8_t valid, int whole, int dec)
{
    char val[16];

    if (valid) snprintf(val, sizeof(val), "%d.%d %%", whole, dec);
    else       snprintf(val, sizeof(val), "--.- %%");

    SSD1306_Clear();
    SSD1306_DrawStringCentered(6,  "HUMIDITY", 2);
    SSD1306_DrawStringCentered(34, val, 3);
    SSD1306_Update();
}

void Link_OnRxByte(uint8_t b)
{
    uint16_t next = (uint16_t)((rx_head + 1U) & (LINK_RX_SIZE - 1U));
    if (next != rx_tail) { rx_ring[rx_head] = b; rx_head = next; }
    else                 { rx_overflow++; }
}

static void handle_frame(const link_data_t *f, DS3231_Time_t rtc_time)
{
    char s[96];
    int hw = f->hum10 / 10, hd = f->hum10 % 10;
    int tt = f->temp10 < 0 ? -f->temp10 : f->temp10;
    const char *tsign = f->temp10 < 0 ? "-" : "";     
    int tw = tt / 10, td = tt % 10;
 
    if (link_seen && (uint16_t)(f->seq - last_seq) != 1U)
        Print("[LINK] frame(s) lost\r\n");
    link_seen = 1; link_ok = 1;
    last_seq = f->seq; last_rx_ms = HAL_GetTick();
 
		if (oled_ok)
		{
				oled_show_humidity(1, hw, hd);
		}                                 
 
    snprintf(s, sizeof(s), "%02d.%02d.%04d %02d:%02d:%02d | Temp: %s%d.%d C, Hum: %d.%d %%\r\n",
             rtc_time.date, rtc_time.month, rtc_time.year,
             rtc_time.hours, rtc_time.minutes, rtc_time.seconds,
             tsign, tw, td, hw, hd);
    Print(s);
 
    if (++log_cnt >= LOG_EVERY_N)
    {
        log_cnt = 0;
        if (!sd_mounted)
            Print("  [SD] no card\r\n");
        else
        {
            snprintf(csv_line, sizeof(csv_line), "%02d.%02d.%04d;%02d:%02d:%02d;%s%d.%d;%d.%d\r\n",
                     rtc_time.date, rtc_time.month, rtc_time.year,
                     rtc_time.hours, rtc_time.minutes, rtc_time.seconds,
                     tsign, tw, td, hw, hd);
						if (sd_append_file("log.csv", csv_line) != 0) 
						{ 
								Print("  [SD] write failed\r\n"); 
								sd_unmount(); 
						}
						else
						{
							Print("  [SD] saved\r\n"); 
						}
        }
    }
}

#if MOCK_RX_ENABLE

static void mock_receive_frame(void)
{
    const char *p = mock_frames[mock_index];

    while (*p)
    {
        Link_OnRxByte((uint8_t)*p);
        p++;
    }

    Link_OnRxByte('\r');
    Link_OnRxByte('\n');

    mock_index++;

    if (mock_index >= MOCK_FRAME_COUNT)
        mock_index = 0;
}

#endif

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
	MX_FATFS_Init();
	
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);
	/* RS-485 receive: USART1 RXNE interrupt (NVIC priority is set in HAL_UART_MspInit) */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	
	if (DS3231_Init() != HAL_OK)
	{
			Print("[RTC] initialization failed\r\n");
	}
	else
	{
    Print("[RTC] initialized\r\n");
	}
	
	oled_ok = (SSD1306_Init() == HAL_OK);
	if (!oled_ok) 
			Print("OLED not found\r\n");
	else
			oled_show_humidity(0, 0, 0);
	
  if (sd_mount() != 0)
      Print("SD mount FAILED\r\n");
  else
      sd_dma_test();
	
	link_rx_init(&link_rx, NODE_ADDR);
	
	Print("System ready.\r\n");
	//DS3231_Time_t init_time = {10, 31, 20, 4, 22, 10, 2026};  
	//DS3231_SetTime(&init_time);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	#if MOCK_RX_ENABLE

    if (!mock_started ||
        (uint32_t)(HAL_GetTick() - mock_last_ms) >= MOCK_PERIOD_MS)
    {
        mock_last_ms = HAL_GetTick();
        mock_started = 1;

        mock_receive_frame();
    }

#endif
		
    while (rx_tail != rx_head)                      
    {
				DS3231_Time_t rtc_time;
        uint8_t b = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) & (LINK_RX_SIZE - 1U));
 
        link_data_t f;
        link_result_t r = link_rx_feed(&link_rx, b, &f);
				if (r == LINK_FRAME)
				{
						if (DS3231_ReadTime(&rtc_time))
						{
								handle_frame(&f, rtc_time);
						}
						else
						{
								Print("[RTC] read error\r\n");
						}
				}
				else if (r == LINK_BAD)
				{
						Print("[LINK] bad frame\r\n");
				}
    }
		
    uint8_t now_ok = link_seen && ((uint32_t)(HAL_GetTick() - last_rx_ms) < LINK_TIMEOUT_MS);
    if (link_ok && !now_ok)
    {
        Print("[LINK] sensor timeout\r\n");
        if (oled_ok)
            oled_show_humidity(0, 0, 0); 
    }
    link_ok = now_ok;

    /* link diagnostics: print only when a counter changed */
    {
        static uint32_t d_ovf = 0, d_err = 0;
        uint32_t o = rx_overflow, e = uart_err;
        if (o != d_ovf || e != d_err)
        {
            char ds[56];
            d_ovf = o; d_err = e;
            snprintf(ds, sizeof(ds), "[LINK] rx overflow=%lu, uart err=%lu\r\n",
                     (unsigned long)o, (unsigned long)e);
            Print(ds);
        }
    }

    /* ===== HOT-SWAP ===== */
		static uint32_t last_sd_check = 0;

		if ((uint32_t)(HAL_GetTick() - last_sd_check) >= 10000U)
		{
				last_sd_check = HAL_GetTick();

				if (sd_mounted)
				{
						if (sd_read_block(dummy, 0) != 0)
						{
								Print("[SD] card removed\r\n");
								sd_unmount();
						}
				}
				else
				{
						Print("[SD] trying to mount card...\r\n");

						if (sd_mount() == 0)
						{
								Print("[SD] card mounted successfully\r\n");
						}
						else
						{
								Print("[SD] mount failed, retrying...\r\n");
						}
				}
		}

    HAL_Delay(10);
  /* USER CODE END 3 */
}}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  * @brief USART1 Initialization Function (RS-485 link to the other board)
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
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
