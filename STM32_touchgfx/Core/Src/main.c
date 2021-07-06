/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "app_touchgfx.h"
#include "delay.h"
#include "image.h"
#include "lcd.h"

#define DMA2D_RTOM (0U)
#define DMA2D_MTOM (1U)
#define DMA2D_BLENDING (2U)
#define DMA2D_MODE (DMA2D_BLENDING)
#define TILE_COUNT_ROW (5U)
#define TILE_WIDTH_PIXEL (95U)

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA2D_Init(void);
static void MX_CRC_Init(void);
static void MX_FMC_Init(void);
uint16_t framebuffer[76800];

#if (DMA2D_MODE == DMA2D_RTOM)
/* DMA 2D R2M */
static inline void DMA2D_Fill(void* pDst, uint32_t width, uint32_t height, uint32_t lineOff, uint32_t pixelFormat, uint32_t color) {
    /* DMA2D配置 */
    DMA2D->CR = 0x00030000UL;                                 // 配置为寄存器到储存器模式
    DMA2D->OCOLR = color;                                     // 设置填充使用的颜色，格式应该与设置的颜色格式相同
    DMA2D->OMAR = (uint32_t)pDst;                             // 填充区域的起始内存地址
    DMA2D->OOR = lineOff;                                     // 行偏移，即跳过的像素，注意是以像素为单位
    DMA2D->OPFCCR = pixelFormat;                              // 设置颜色格式
    DMA2D->NLR = (uint32_t)(width << 16) | (uint16_t)height;  // 设置填充区域的宽和高，单位是像素

    /* 启动传输 */
    DMA2D->CR |= DMA2D_CR_START;

    /* 等待DMA2D传输完成 */
    while (DMA2D->CR & DMA2D_CR_START) {
    }
}

void FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    void* pDist = &(((uint16_t*)framebuffer)[y * 320 + x]);
    DMA2D_Fill(pDist, w, h, 320 - w, 0x2, color);
}
#elif (DMA2D_MODE == DMA2D_MTOM)
static void DMA2D_MemCopy(uint32_t pixelFormat, void* pSrc, void* pDst, int xSize, int ySize, int OffLineSrc, int OffLineDst) {
    /* DMA2D配置 */
    DMA2D->CR = 0x00000000UL;
    DMA2D->FGMAR = (uint32_t)pSrc;
    DMA2D->OMAR = (uint32_t)pDst;
    DMA2D->FGOR = OffLineSrc;
    DMA2D->OOR = OffLineDst;
    DMA2D->FGPFCCR = pixelFormat;
    DMA2D->NLR = (uint32_t)(xSize << 16) | (uint16_t)ySize;
    DMA2D->OPFCCR = pixelFormat;
    /* 启动传输 */
    DMA2D->CR |= DMA2D_CR_START;

    /* 等待DMA2D传输完成 */
    while (DMA2D->CR & DMA2D_CR_START) {
    }
}

static void DMA2D_DisplayFrameAt(uint16_t index) {
    uint16_t* pStart = (uint16_t*)image_big;
    void* pDist = (uint16_t*)framebuffer;
    pStart += (index / TILE_COUNT_ROW) * (TILE_WIDTH_PIXEL * TILE_WIDTH_PIXEL * TILE_COUNT_ROW);
    pStart += (index % TILE_COUNT_ROW) * TILE_WIDTH_PIXEL;
    uint32_t offlineSrc = (TILE_COUNT_ROW - 1) * TILE_WIDTH_PIXEL;
    uint32_t offlineDist = 0;

    DMA2D_MemCopy(0x2, (void*)pStart, pDist, TILE_WIDTH_PIXEL, TILE_WIDTH_PIXEL, offlineSrc, offlineDist);
}
#elif (DMA2D_MODE == DMA2D_BLENDING)
void DMA2D_MixColors(void* pFg, void* pBg, void* pDst,
                      uint32_t offlineFg, uint32_t offlineBg, uint32_t offlineDist,
                      uint16_t xSize, uint16_t ySize,
                      uint32_t pixelFormat, uint8_t opa) {
    DMA2D->CR = 0x00020000UL;  // 设置工作模式为存储器到存储器并带颜色混合

    DMA2D->FGMAR = (uint32_t)pFg;  // 设置前景数据内存地址
    DMA2D->BGMAR = (uint32_t)pBg;  // 设置背景数据内存地址
    DMA2D->OMAR = (uint32_t)pDst;  // 设置数据输出内存地址

    DMA2D->FGOR = offlineFg;   // 设置前景数据传输偏移
    DMA2D->BGOR = offlineBg;   // 设置背景数据传输偏移
    DMA2D->OOR = offlineDist;  // 设置数据输出传输偏移

    DMA2D->NLR = (uint32_t)(xSize << 16) | (uint16_t)ySize;  // 设置图像数据宽高（像素）

    DMA2D->FGPFCCR = pixelFormat               // 设置前景色颜色格式
                     | (1UL << 16)             // 忽略前景颜色数据中的Alpha通道
                     | ((uint32_t)opa << 24);  // 设置前景色不透明度

    DMA2D->BGPFCCR = pixelFormat;  // 设置背景颜色格式
    DMA2D->OPFCCR = pixelFormat;   // 设置输出颜色格式

    /* 启动传输 */
    DMA2D->CR |= DMA2D_CR_START;

    /* 等待DMA2D传输完成 */
    while (DMA2D->CR & DMA2D_CR_START) {
    }
}
#endif

/* USER CODE BEGIN PFP */

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

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();
    delay_init(180);
    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA2D_Init();
    MX_CRC_Init();
    MX_FMC_Init();
    LCD_Init();
    //MX_TouchGFX_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        //MX_TouchGFX_Process();
        // 填充背景色
#if (DMA2D_MODE == DMA2D_RTOM)
        FillRect(0, 0, 320, 240, WHITE);

        // 绘制数据条
        FillRect(80, 80, 20, 120, 0x001f);
        FillRect(120, 100, 20, 100, 0x001f);
        FillRect(160, 40, 20, 160, 0x001f);
        FillRect(200, 60, 20, 140, 0x001f);
        // 绘制X轴
        FillRect(40, 200, 240, 1, 0x0000);
        LCD_update_framebuffer(0, 0, 320, 240, framebuffer);

        while (1)
            ;
#elif (DMA2D_MODE == DMA2D_MTOM)
        while (1) {
            for (int i = 0; i < 25; i++) {
                DMA2D_DisplayFrameAt(i);
                delay_ms(100);
                LCD_update_framebuffer(0, 0, 95, 95, framebuffer);
            }
        }
#elif (DMA2D_MODE == DMA2D_BLENDING)
        while (1) {
            for (uint8_t i = 0; i < 255; i++) {
                /* code */
                DMA2D_MixColors((void *)cat, (void *)fox, framebuffer, 0, 0, 0, 240, 190, 0x2, i);
                LCD_update_framebuffer(0, 0, 240, 190, framebuffer);
                delay_ms(10);
            }
        }
#endif

        /* USER CODE BEGIN 3 */

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
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Activate the Over-Drive mode
  */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void) {
    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance = CRC;
    if (HAL_CRC_Init(&hcrc) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void) {
    /* USER CODE BEGIN DMA2D_Init 0 */

    /* USER CODE END DMA2D_Init 0 */

    /* USER CODE BEGIN DMA2D_Init 1 */

    /* USER CODE END DMA2D_Init 1 */
    hdma2d.Instance = DMA2D;
    hdma2d.Init.Mode = DMA2D_RTOM;
    hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
    hdma2d.Init.OutputOffset = 0;
    hdma2d.LayerCfg[1].InputOffset = 0;
    hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
    hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
    hdma2d.LayerCfg[1].InputAlpha = 0;

    if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK) {
        Error_Handler();
    }

    /* USER CODE BEGIN DMA2D_Init 2 */

    /* USER CODE END DMA2D_Init 2 */
}

/* FMC initialization function */
static void MX_FMC_Init(void) {
    /* USER CODE BEGIN FMC_Init 0 */

    /* USER CODE END FMC_Init 0 */

    FMC_NORSRAM_TimingTypeDef Timing = {0};
    FMC_NORSRAM_TimingTypeDef ExtTiming = {0};

    /* USER CODE BEGIN FMC_Init 1 */

    /* USER CODE END FMC_Init 1 */

    /** Perform the SRAM1 memory initialization sequence
  */
    hsram1.Instance = FMC_NORSRAM_DEVICE;
    hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
    /* hsram1.Init */
    hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
    hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
    hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
    hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
    hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
    hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram1.Init.WrapMode = FMC_WRAP_MODE_DISABLE;
    hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
    hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
    hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
    hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_ENABLE;
    hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
    hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
    hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
    /* Timing */
    Timing.AddressSetupTime = 15;
    Timing.AddressHoldTime = 15;
    Timing.DataSetupTime = 70;
    Timing.BusTurnAroundDuration = 0;
    Timing.CLKDivision = 16;
    Timing.DataLatency = 17;
    Timing.AccessMode = FMC_ACCESS_MODE_A;
    /* ExtTiming */
    ExtTiming.AddressSetupTime = 15;
    ExtTiming.AddressHoldTime = 15;
    ExtTiming.DataSetupTime = 15;
    ExtTiming.BusTurnAroundDuration = 0;
    ExtTiming.CLKDivision = 16;
    ExtTiming.DataLatency = 17;
    ExtTiming.AccessMode = FMC_ACCESS_MODE_A;

    if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK) {
        Error_Handler();
    }

    /* USER CODE BEGIN FMC_Init 2 */

    /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(T_CS_GPIO_Port, T_CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(T_SCL_GPIO_Port, T_SCL_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(T_MISOG3_GPIO_Port, T_MISOG3_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : T_CS_Pin */
    GPIO_InitStruct.Pin = T_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(T_CS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : T_MISO_Pin */
    GPIO_InitStruct.Pin = T_MISO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(T_MISO_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : T_SCL_Pin */
    GPIO_InitStruct.Pin = T_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(T_SCL_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : T_PEN_Pin */
    GPIO_InitStruct.Pin = T_PEN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(T_PEN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : T_MISOG3_Pin */
    GPIO_InitStruct.Pin = T_MISOG3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(T_MISOG3_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LCD_BL_Pin */
    GPIO_InitStruct.Pin = LCD_BL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

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

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
