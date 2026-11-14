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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed))
{
  uint8_t head;
  uint8_t run;
  uint8_t dir;
  uint8_t step_l;
  uint8_t step_h;
  uint8_t check;
} CmdPacket;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int8_t step =0;
static uint32_t  lastclick = 0;
uint8_t receivedata[50];
volatile uint8_t dir_flag   = 0;
volatile uint8_t pause_flag = 0;
volatile static uint32_t  last_dir_tick   = 0;
volatile static uint32_t  last_pause_tick = 0;
volatile static uint32_t  run = 1;
volatile static uint32_t  dir = 1;
volatile uint8_t rx_flag  =0;
static uint8_t  rx_pkt[6];              // 回调里拷出的一帧，主循环只读这个
static uint16_t step_ms = 200;          // 当前节拍(ms)，串口可改
static uint8_t  tx_busy = 0;            // 上一帧还没发完
static char tx_buf[48];                 // 回传缓冲区
static uint8_t  need_tx = 0;            // 有状态变化待回传
static uint8_t  need_oled = 1;          // 有状态变化待整体刷新
static uint8_t  need_led_line = 0;      // 只有 LED 行变了，局部刷新

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 把当前状态刷到 OLED 四行，定宽补空格避免旧字符残留 */
static void OLED_ShowState(void)
{
  char text[20];
  char line[32];      /* 16 显示列 + 余量，避免 -Wformat-truncation 报警 */

  /* 先拼出内容，再整体补齐到 16 列：
     长的用空格顶掉残留，短的补空格，保证每次都是完整 16 列 */
  snprintf(text, sizeof(text), "STATE: %s", run ? "RUN" : "PAUSE");
  snprintf(line, sizeof(line), "%-16s", text);
  OLED_ShowString(1, 1, line);

  snprintf(text, sizeof(text), "DIR: %s", dir ? "FWD" : "REV");
  snprintf(line, sizeof(line), "%-16s", text);
  OLED_ShowString(2, 1, line);

  snprintf(text, sizeof(text), "STEP: %ums", (unsigned)step_ms);
  snprintf(line, sizeof(line), "%-16s", text);
  OLED_ShowString(3, 1, line);

  snprintf(text, sizeof(text), "LED: %u", (unsigned)(step + 1));
  snprintf(line, sizeof(line), "%-16s", text);
  OLED_ShowString(4, 1, line);
}

/* 只刷第 4 行 LED：
   流水灯每一步只改 LED 行，整屏重刷一遍要 1024 次 I2C 事务，
   在 200ms 节拍下会把主循环堵死，所以切灯走这个轻量版。 */
static void OLED_ShowLedLine(void)
{
  char text[20];
  char line[32];

  snprintf(text, sizeof(text), "LED: %u", (unsigned)(step + 1));
  snprintf(line, sizeof(line), "%-16s", text);
  OLED_ShowString(4, 1, line);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* LED 高电平点亮：先把三颗全拉低，再点亮第一颗，
     这样开机瞬间屏上的 "LED: 1" 和实际亮着的灯就是一致的 */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  memset(receivedata, 0, sizeof(receivedata));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,receivedata,sizeof(receivedata));

  OLED_Init();
  OLED_ShowState();
  need_oled = 0;      /* 上面已经整屏刷过一次，别让主循环再白刷一遍 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_time = HAL_GetTick();

    /* 1. 消费方向事件 */
    if (dir_flag)
    {
      dir_flag = 0;
      dir = 1 - dir;              // 翻转 dir
      lastclick = now_time; // 方向变了重新计时
      need_tx = 1;
      need_oled = 1;
    }

    /* 2. 消费暂停事件 */
    if (pause_flag)
    {
      pause_flag = 0;
      run = 1 - run;              // 翻转 run
      if (run) { lastclick = now_time; }   // 只有"恢复"才重置
      need_tx = 1;
      need_oled = 1;
    }

    /* 2.5 消费串口事件：解析 + 校验 + 应用 */
    if (rx_flag)
    {
      rx_flag = 0;

      /* 回调里已经把这一帧拷进 rx_pkt 并立刻重启了 DMA，
         这里直接读 rx_pkt，不会和下一帧抢同一个缓冲区 */
      uint8_t buf[6];
      memcpy(buf, rx_pkt, 6);

      CmdPacket *pkt = (CmdPacket *)buf;
      uint8_t sum = (uint8_t)(pkt->head + pkt->run + pkt->dir
                              + pkt->step_l + pkt->step_h);
      uint16_t new_step = (uint16_t)pkt->step_l | ((uint16_t)pkt->step_h << 8);

      /* 帧头对、校验对、字段合法才应用；否则原样丢弃 */
      if (pkt->head == 0xAA && sum == pkt->check
          && pkt->run <= 1 && pkt->dir <= 1
          && new_step >= 100 && new_step <= 2000)
      {
        run     = pkt->run;
        /* 协议里 DIR=00 是正向、01 是反向；内部 dir=1 才是正向，要反过来 */
        dir     = (pkt->dir == 0) ? 1 : 0;
        step_ms = new_step;
        lastclick = now_time;   // 换了节拍，重新计时
        need_tx = 1;
        need_oled = 1;
      }
    }

    /* 3. 只在运行时推进 */
    if (run)
    {
      if (now_time - lastclick >= step_ms)
      {
        lastclick += step_ms;

        // 先全部熄灭（高电平点亮，所以熄灭=拉低）
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

        // 再按 step 点亮对应的灯（高电平点亮）
        if (step == 0)
        {
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        }
        else if (step == 1)
        {
          HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        }
        else if (step == 2)
        {
          HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
        }

        if (dir)
        {
          step++;        // 正向
        }
        else
        {
          step--;        // 反向
        }

        if (step > 2) step = 0;
        if (step < 0) step = 2;

        need_led_line = 1;   // 只改了 LED，局部刷第 4 行
     }
    }

    /* 4. 状态有变化就回传一帧 */
    if (need_tx && !tx_busy)
    {
      need_tx = 0;
      int n = snprintf(tx_buf, sizeof(tx_buf),
                       "STATE=%s,DIR=%s,STEP=%u,LED=%u\r\n",
                       run ? "RUN" : "PAUSE",
                       dir ? "FWD" : "REV",
                       (unsigned)step_ms,
                       (unsigned)(step + 1));
      if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)tx_buf, (uint16_t)n) == HAL_OK)
      {
        tx_busy = 1;
      }
      else
      {
        need_tx = 1;   // 串口忙，下轮再试
      }
    }

    /* 5. 状态有变化就刷屏 */
    if (need_oled)
    {
      need_oled = 0;
      need_led_line = 0;          // 整屏刷新已经把 LED 行带上了
      OLED_ShowState();
    }
    else if (need_led_line)
    {
      need_led_line = 0;
      OLED_ShowLedLine();         // 只改了 LED 行
    }


  /* USER CODE END 3 */

    }

}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now_time = HAL_GetTick();


  if (GPIO_Pin==KEY_DIR_Pin)
  {
    if (now_time - last_dir_tick >= 20)
    {
      last_dir_tick = now_time;

      dir_flag =1;
    }
  }

  else if (GPIO_Pin==KEY_PAUSE_Pin)
  {
    if (now_time - last_pause_tick >= 20)
    {
      last_pause_tick = now_time;
      pause_flag =1;
    }
  }
}

/* 串口 DMA 空闲中断回调：只记标志，解析交给主循环 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART1) return;

  /* 半传输也会进这里，那不是"收完一包"，直接忽略 */
  if (huart->RxEventType != HAL_UART_RXEVENT_IDLE) return;

  /* 用 >= 而不是 ==：USB-TTL 或串口助手有时会把 6 字节拆成两次空闲事件，
     写死 == 6 会两包全丢。解析端只取前 6 字节，多收的字节不影响。 */
  if (Size >= 6)
  {
    /* 关键：先在回调里把这一帧拷走，再重启 DMA。
       如果等主循环去读 receivedata，那期间 DMA 已经在往同一个缓冲区写下一帧，
       拷到一半就会被新数据覆盖，校验随机失败。 */
    memcpy(rx_pkt, receivedata, 6);
    rx_flag = 1;
  }

  /* 必须重新启动，否则只收得到一包 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receivedata, sizeof(receivedata));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    tx_busy = 0;
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
