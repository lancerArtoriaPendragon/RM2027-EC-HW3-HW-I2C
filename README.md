# 电控组第三次培训作业
- **陈金泽**
- ***20263006501***

> 本仓库是作业的**选做版**：OLED 改用 I2C1 外设驱动。
> 必做项的软件 I²C 版在另一个仓库 `RM2027-EC-HW3-SW-I2C`，按作业要求两份代码都要提交。
> 两版差异与踩坑详解见同目录 `硬件I2C实现说明.md`。

## 环境

| 项目 | 版本 / 型号 |
|---|---|
| MCU | STM32F103C8T6，LQFP48，64KB Flash / 20KB RAM |
| 时钟 | HSE 8MHz 晶振 → PLL×9 → HCLK 72MHz，APB1 36MHz，APB2 72MHz，Flash Latency 2 |
| 代码生成 | STM32CubeMX 6.15.0 + STM32Cube FW_F1 V1.8.7 |
| IDE | CLion 2026.2.2 |
| 编译器 | arm-none-eabi-gcc 10.3.1（STM32CubeCLT 1.22.0） |
| 构建 | CMake + Ninja |
| 下载调试 | ST-Link V2（FW V2J37S7），SWD |
| 显示 | 0.96 寸 SSD1306 OLED，**硬件 I²C（I2C1 外设）** |

外设全部由 CubeMX 生成初始化代码，业务逻辑只写在 `/* USER CODE BEGIN */` 保护区内，重新 GENERATE 不会丢。

## 工程

```
RM2027-EC-HW3 - 硬件I2C/
├── Core/
│   ├── Inc/  main.h  gpio.h  usart.h  dma.h  i2c.h  OLED.h  OLED_Font.h
│   └── Src/  main.c  gpio.c  usart.c  dma.c  i2c.c  OLED.c  stm32f1xx_it.c
├── Drivers/                 HAL 库 + CMSIS
├── cmake/  CMakeLists.txt  CMakePresets.json
├── RM2027-EC-HW3.ioc        CubeMX 工程文件
├── 硬件I2C实现说明.md        软硬件 I²C 对比与踩坑详解
├── STM32F103XX_FLASH.ld
└── startup_stm32f103xb.s
```

引脚分配：

| 用途 | 引脚 | 配置 |
|---|---|---|
| LED1 / LED2 / LED3 | PB12 / PB13 / PB14 | 推挽输出，**高电平点亮**（外接 220Ω 限流） |
| KEY_DIR（切方向） | PB10 | 上拉输入 + 下降沿 EXTI，EXTI15_10_IRQn |
| KEY_PAUSE（暂停/继续） | PB11 | 同上 |
| USART1 TX / RX | PA9 / PA10 | 115200 8N1，DMA1_Channel4(TX) / DMA1_Channel5(RX) + IDLE 空闲中断 |
| OLED SCL / SDA | **PB6 / PB7** | **硬件 I²C1**，复用开漏，100kHz，从机地址 0x78 |

工程名 `RM2027-EC-HW3`，可执行文件 `RM2027-EC-HW3.elf`。

## 实现思路

整个程序是「中断只记事件、主循环统一处理」的结构。EXTI 回调和串口回调里只置标志位，所有状态修改、GPIO 输出、OLED 刷新、串口回传都发生在 `while(1)` 里，因此按键和串口操作的是同一份状态，不存在两边各改一半的问题。

**状态机**。三个持久状态加一个位置：

- `run`：0 = 暂停，1 = 运行（是否推进流水灯）
- `dir`：1 = 正向，0 = 反向
- `step_ms`：当前节拍毫秒数，100~2000
- `step`：`int8_t`，0/1/2，当前亮第几颗灯
- 两个事件标志 `dir_flag` / `pause_flag`，都是 `volatile uint8_t`

事件标志的消费放在 `while(1)` **最前面**，先清零再做动作。放后面或者放在推进逻辑之后，暂停状态下按方向键要等下一轮才生效，手感上就是「按了没反应」。

**方向推进**用带符号的加减，不用取模：

```c
if (dir) { step++; } else { step--; }
if (step > 2) step = 0;
if (step < 0) step = 2;
```

不能写 `step = (step + 1) % 3` 这类写法做反向，C 里 `-1 % 3 == -1`，越界兜底会失效。两行 `if` 是最直白也最不容易错的方式。

**暂停语义**。暂停时推进分支整段不执行，一行 GPIO 都不写，当前亮着的灯就保持原样，符合「暂停时保留当前 LED」。恢复时（`run` 由 0 变 1）必须重置时间基准 `lastclick = HAL_GetTick();`。不重置的话，暂停期间 `lastclick` 一直不动，`now - lastclick` 会累积成几千毫秒，一恢复 `lastclick += step_ms` 就连着成立二十来次，灯会在一瞬间闪过两三颗。

**串口协议**。6 字节定长包：

```
AA  RUN  DIR  STEP_L  STEP_H  CHECK
```

- `RUN`：00 = 暂停，01 = 运行
- `DIR`：00 = 正向，01 = 反向
- `STEP`：100~2000 ms，**低字节在前**（小端）
- `CHECK = (0xAA + RUN + DIR + STEP_L + STEP_H) % 256`

例：运行正向 300ms → `AA 01 00 2C 01 D8`；暂停反向 1000ms → `AA 00 01 E8 03 96`。

协议里的 DIR 语义和内部 `dir` 是相反的（协议 00 才是正向），解析时必须做一次映射：`dir = (pkt->dir == 0) ? 1 : 0;`。这一步漏掉，串口设正向、灯却反着跑。

接收用 DMA + IDLE 空闲中断：`HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receivedata, sizeof(receivedata))` 启动，回调里判断事件类型、拷贝一帧、重启 DMA。校验放在主循环做：帧头 `0xAA`、累加和、`run <= 1 && dir <= 1`、`step_ms` 落在 100~2000，任一不满足就整包丢弃，不碰任何状态。

**回传**。状态应用后回传一帧，按键改 RUN/DIR 也回传，切灯不回传（否则 200ms 一帧会把串口刷满）：

```
STATE=RUN,DIR=FWD,STEP=200,LED=1\r\n
```

用 `HAL_UART_Transmit_IT()` 发送，配 `tx_busy` 标志防重入，`HAL_UART_TxCpltCallback` 里清标志。

**OLED 刷新**。四行定宽显示，每行用 `snprintf` 拼出内容再 `%-16s` 补满 16 列，保证新内容比旧的短时旧字符被空格顶掉：

```
STATE: RUN
DIR: FWD
STEP: 200ms
LED: 1
```

刷新分两级：状态（RUN/DIR/STEP）变化走整屏 `OLED_ShowState()`；只有 LED 变走 `OLED_ShowLedLine()` 只刷第 4 行，原因见下一节的性能账。

### 硬件 I²C 驱动

本版是作业的**选做项**：OLED 改用 I2C1 外设驱动，而不是 GPIO 手动翻转。改动只在 `Core/Src/OLED.c` 头部的两个函数，上层一行没动：

```c
#include "i2c.h"                    /* 取 hi2c1 句柄 */

#define OLED_I2C_ADDR     0x78      /* 0x3C << 1 */
#define OLED_I2C_TIMEOUT  10        /* ms，绝不用 HAL_MAX_DELAY */

void OLED_WriteCommand(uint8_t Command)
{
    uint8_t buf[2] = {0x00, Command};   /* 0x00 = 后面是命令 */
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, buf, 2, OLED_I2C_TIMEOUT);
}

void OLED_WriteData(uint8_t Data)
{
    uint8_t buf[2] = {0x40, Data};      /* 0x40 = 后面是显存数据 */
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, buf, 2, OLED_I2C_TIMEOUT);
}
```

`OLED_SetCursor` / `OLED_Clear` / `OLED_ShowChar` / `OLED_ShowString` 等全部函数只间接调用这两个函数，所以换掉它们就整体切到了硬件 I²C，驱动文件从 350 行缩到 286 行。

三个必须注意的点：

- **地址传 `0x78` 不是 `0x3C`**。SSD1306 的 7 位地址是 `0x3C`，但 `HAL_I2C_Master_Transmit` 要求传左移一位后的 8 位形式，`0x3C << 1 = 0x78`。传 `0x3C` 会直接被 NACK。
- **`Timeout` 给 10ms，绝不用 `HAL_MAX_DELAY`**。F1 的硬件 I²C 有勘误：总线被拉死时 `SR2` 的 `BUSY` 位可能卡住不退，`HAL_MAX_DELAY` 会让主循环永久阻塞。
- **必须有外部上拉电阻**。`GPIO_MODE_AF_OD` 在 F1 的 HAL 里同样不读 `Pull` 字段（`stm32f1xx_hal_gpio.c:232-235`），内部上拉完全不生效。模块板上已焊 4.7k 就不用外接，判断方法：断电用万用表蜂鸣档量模块 `SCL 对 VCC`、`SDA 对 VCC`，响就是自带。

详细对比（软件 vs 硬件 I²C 的原理、优劣、踩坑、标准库写法对照）见同目录 `硬件I2C实现说明.md`。

### 踩过的坑

1. **LED 极性反了**。接线是 `PB12 → 220Ω → LED → GND`，属于高电平点亮。代码最初按低电平点亮写（全灭发 SET、点亮发 RESET），现象是每次亮两颗：`LED1` 时黄红、`LED2` 时红蓝、`LED3` 时蓝黄。改成全灭发 RESET、点亮发 SET 后正常。另外开机在 `USER CODE BEGIN 2` 里先把三灯拉低再点亮第一颗，否则屏上已经写着 `LED: 1`，灯还全灭着，有半秒的空档。

2. **OLED 全黑只亮电源灯**。软件版驱动里硬编码的是 `PB0/PB1`，实物接在 `PB6/PB7`，代码一直在对着两根空脚翻转。教训：驱动默认引脚 ≠ 你的接线，先对一遍再怀疑时序。切到硬件 I²C 后引脚变成 PB6/PB7，与实物一致。

3. **`Pull = GPIO_PULLUP` 对开漏输出无效**。`stm32f1xx_hal_gpio.c` 里 `GPIO_MODE_OUTPUT_OD` 分支只做 `config = Speed + GPIO_CNF_GP_OUTPUT_OD`，**根本不读 Pull 字段**（只有 INPUT / EXTI 分支才读）。`GPIO_MODE_AF_OD` 分支同样不读。所以开漏输出没有任何上拉能力，模块上没焊 4.7k 时 SCL 永远回不到高电平，一个时钟都发不出去。软件版的绕法是「写 1 时把引脚切成输入 + 内部上拉释放总线，写 0 时才切开漏真正拉低」，见软件版 `OLED.c` 的 `OLED_PinWrite()`；**硬件 I²C 没有这个绕法，必须外接 4.7k 上拉到 3V3**。

4. **CubeMX `USER CODE` 保护区边界**。`while(1)` 的闭合 `}` 位于 `USER CODE BEGIN 3` 与 `USER CODE END 3` **之间**。把业务代码写在 `END WHILE` 和 `BEGIN 3` 之间，重新 GENERATE 会被整段覆盖（实测 `main.c` 从 5885 B 缩到 5106 B，103–137 行全部消失）；反过来把 `BEGIN 3` 往上挪，CubeMX 会把模板自带的 `}` 当垃圾删掉，编译报 `error: expected declaration or statement at end of input`。正确形状是 `BEGIN 3` → 业务代码 → `}`（闭 while）→ `END 3` → `}`（闭 main）。

5. **串口助手默认 9600 收不到数据**，要改成 115200；接收模式选文本，发送模式选 HEX。

6. **烧录报 `Error: Unable to get core ID` / `No STM32 target found!`**。ST-Link 能读到 3.30V，但那个电压是 ST-Link 自己量的，芯片没供电就不会响应 SWD。先查板子 USB 有没有插。

7. **`HAL_UART_Transmit_IT` 忙时直接返回 `HAL_BUSY` 且不排队**，不用 `tx_busy` 标志的话，连按两下按键回传就丢帧。

## 非阻塞延时

流水灯节拍不能用 `HAL_Delay()`。`HAL_Delay()` 是忙等，200ms 里 CPU 什么都不能干，按键响应和串口接收都会被拖住——而作业要求「暂停期间按键和串口仍有效」，一旦阻塞就做不到。

这里用 `HAL_GetTick()` 做时间戳差值：

```c
static uint32_t lastclick = 0;
...
if (HAL_GetTick() - lastclick >= step_ms)
{
    lastclick += step_ms;
    /* 推进流水灯 */
}
```

四个细节都不能省：

- **必须是 `static uint32_t`**。`HAL_GetTick()` 返回 `uint32_t`，约 49.7 天回绕一次。无符号整数相减在回绕时结果仍然正确（模 2³² 运算），换成有符号或 `int` 就会在回绕点算出一个负数，节拍直接停掉。
- **必须写成相减形式**。`HAL_GetTick() - lastclick >= step_ms` 而不是 `HAL_GetTick() >= lastclick + step_ms`，后者在 `lastclick + step_ms` 溢出时判据就废了。
- **推进用 `lastclick += step_ms`，不是 `lastclick = HAL_GetTick()`**。后者把每轮循环的执行耗时也算进去，误差会一轮一轮累积成漂移；前者把基准钉在理想节拍上，长时间跑下来平均周期还是准的。
- **暂停/恢复、改节拍时都要重置基准**。这是前面提到的「恢复瞬间连切」的根因。

标准库时代这套写法是直接读全局变量 `uwTick`，HAL 只是把它封装成了 `HAL_GetTick()`，原理完全一样。

**OLED 整屏刷新为什么要拆**（这是实测踩到的坑）。一次整屏刷新 = 4 行 × 16 列 = 64 个字符 × 每个字符 16 次 `OLED_WriteData` = **1024 次 I²C 事务**，每次事务约 290µs，合计阻塞主循环约 **300ms**。节拍 500ms 时还能勉强撑住；改成 200ms 后，主循环被堵 300ms，出来时 `now - lastclick` 已经跨过好几个节拍，`lastclick += step_ms` 连续成立，灯一下跳两三颗。

解法是把刷新拆成两级：切灯只调 `OLED_ShowLedLine()` 刷第 4 行（约 75ms），加一个状态位 `need_led_line`；主循环里先看整屏标志，没有整屏需求才做局部刷新：

```c
if (need_oled)      { need_oled = 0; need_led_line = 0; OLED_ShowState(); }
else if (need_led_line) { need_led_line = 0; OLED_ShowLedLine(); }
```

## 核心代码

主循环骨架：事件消费 → 推进 → 回传 → 刷新，四段严格按顺序。

```c
while (1)
{
  uint32_t now_time = HAL_GetTick();

  if (dir_flag)   { dir_flag = 0;   dir = 1 - dir;  lastclick = now_time; need_tx = 1; need_oled = 1; }
  if (pause_flag) { pause_flag = 0; run = 1 - run;
                    if (run) { lastclick = now_time; }   /* 只有恢复才重置 */
                    need_tx = 1; need_oled = 1; }
  if (rx_flag)    { /* 见下方串口解析 */ }

  if (run && now_time - lastclick >= step_ms)
  {
    lastclick += step_ms;
    HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);  /* 高电平点亮，先全灭 */
    if (step == 0)      HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
    else if (step == 1) HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
    else if (step == 2) HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_SET);
    if (dir) { step++; } else { step--; }
    if (step > 2) step = 0;
    if (step < 0) step = 2;
    need_led_line = 1;
  }
  /* 4. need_tx 回传  5. need_oled / need_led_line 刷新 */
}
```

EXTI 回调：只做消抖和置标志，不碰任何状态。

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now_time = HAL_GetTick();

  if (GPIO_Pin == KEY_DIR_Pin)
  {
    if (now_time - last_dir_tick >= 20)   /* 20ms 消抖窗口 */
    {
      last_dir_tick = now_time;
      dir_flag = 1;                       /* 主循环消费并清零 */
    }
  }
  else if (GPIO_Pin == KEY_PAUSE_Pin)
  {
    if (now_time - last_pause_tick >= 20)
    {
      last_pause_tick = now_time;
      pause_flag = 1;
    }
  }
}
```

串口解析：三重校验全过才应用，否则整包丢弃。

```c
CmdPacket *pkt = (CmdPacket *)buf;
uint8_t  sum      = (uint8_t)(pkt->head + pkt->run + pkt->dir + pkt->step_l + pkt->step_h);
uint16_t new_step = (uint16_t)pkt->step_l | ((uint16_t)pkt->step_h << 8);

if (pkt->head == 0xAA && sum == pkt->check
    && pkt->run <= 1 && pkt->dir <= 1
    && new_step >= 100 && new_step <= 2000)
{
  run     = pkt->run;
  dir     = (pkt->dir == 0) ? 1 : 0;   /* 协议 00=正向，内部 dir=1 才是正向 */
  step_ms = new_step;
  lastclick = now_time;                /* 换了节拍，重新计时 */
  need_tx = 1;
  need_oled = 1;
}
```

DMA 空闲回调：拷一帧再重启 DMA，避免和下一帧抢缓冲区。

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART1) return;
  /* 半传输中断也会进来，那不是"收完一包" */
  if (huart->RxEventType != HAL_UART_RXEVENT_IDLE) return;

  if (Size >= 6)          /* 拆包时会分两次空闲事件，写死 == 6 会两包全丢 */
  {
    memcpy(rx_pkt, receivedata, 6);   /* 先拷走，主循环只读 rx_pkt */
    rx_flag = 1;
  }
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receivedata, sizeof(receivedata));  /* 必须重启 */
}
```

## 编译结果

```
Memory region         Used Size  Region Size  %age Used
             RAM:        2568 B        20 KB     12.54%
           FLASH:       19900 B        64 KB     30.36%
```

比软件版（18528 B）多 1372 B，因为链接了 `stm32f1xx_hal_i2c.c`（硬件 I²C 的 HAL 驱动）。RAM 完全一样，业务代码一行没改。

唯一 warning 来自驱动自带字模，不是用户代码：

```
Core/Inc/OLED_Font.h:8:1: warning: missing braces around initializer [-Wmissing-braces]
```

## 运行效果

上电后三颗 LED 单灯流水（同一时刻只亮一颗），PB10 切换正/反向、PB11 暂停/继续，暂停时灯停在原位不动，按键和串口全程有效；串口按 6 字节协议下发参数后，OLED 四行与 LED 状态同步更新，并按 `STATE=...,DIR=...,STEP=...,LED=...` 回传一帧。

演示视频见仓库根目录：`陈金泽_第三次培训作业.mp4`。
