# 硬件 I²C 驱动 OLED 实现说明

- **陈金泽**
- ***20263006501***

芯片：STM32F103C8T6（Cortex-M3，72MHz）
显示屏：0.96 寸 SSD1306 OLED，128×64，四针 I²C 接口

本文对应作业的选做项。必做项的软件 I²C 版本单独提交一份代码，本文只讲两版的差异，以及硬件版在实现过程中实际遇到的问题。

---

## 一、两种 I²C 的本质区别

I²C 是一套电气时序规范：两根线（SCL 时钟、SDA 数据）、开漏输出、外部上拉、起始/停止条件、每字节 9 个时钟（8 位数据 + 1 位应答）。区别只在于**这套时序由谁来产生**。

**软件 I²C**：CPU 用两个普通 GPIO，按规范手动翻转电平。发一位就是「把 SDA 设成这一位的值 → 拉高 SCL → 拉低 SCL」，全在 `for` 循环里打拍子。时序精度取决于代码，CPU 在整段传输期间被完全占用。

**硬件 I²C**：I2C1 是芯片内部的专用外设，自带移位寄存器、波特率发生器、状态机和引脚复用。CPU 只要把要发的字节写进数据寄存器 `I2C1->DR`，外设自动产生 START、逐位输出、在第 9 个时钟采样 ACK、最后产生 STOP。CPU 在传输期间可以去干别的。

两者在**协议层完全等价**——从机（SSD1306）看到的波形是同一种东西，所以同一块屏幕、同一套初始化命令、同一套字库代码都能直接复用。差别只在**时序由谁生成**，以及由此带来的引脚、CPU 占用、速率、可靠性差异。

这也决定了本次移植的性质：不是重写驱动，而是把「产生时序的那一层」换掉。第 4 节会看到，上层的字符显示、数字显示、清屏函数一行都没动。

---

## 二、软件 I²C 的实现与三个坑

### 2.1 时序是怎么打出来的

驱动文件是 `Core/Src/OLED.c`，时序由三个手写函数构成：

```c
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SDA(0);		/* SCL 高电平期间 SDA 由高变低 = 起始条件 */
	OLED_W_SCL(0);
}

void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	OLED_W_SDA(1);		/* SCL 高电平期间 SDA 由低变高 = 停止条件 */
}

void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(Byte & (0x80 >> i));	/* 先摆好数据 */
		OLED_W_SCL(1);					/* 再拉高时钟，从机在此刻采样 */
		OLED_W_SCL(0);
	}
	OLED_W_SCL(1);	/* 第 9 个时钟：应答位，本项目直接丢弃不处理 */
	OLED_W_SCL(0);
}
```

一帧写命令 = `0x78`（从机地址）+ `0x00`（控制字节，表示后面是命令）+ 命令本身。
一帧写数据 = `0x78` + `0x40`（控制字节，表示后面是显存数据）+ 数据。

```c
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);
	OLED_I2C_SendByte(0x00);
	OLED_I2C_SendByte(Command);
	OLED_I2C_Stop();
}
```

注意第 9 个时钟的处理方式：`OLED_I2C_SendByte()` 只发了一个空时钟，没有读 SDA 判断从机是否应答。这是原厂驱动的写法，对单个从机、短线的场景能跑，但意味着**从机不在线时程序不会知道**，屏幕黑着也不报错。这是后面几个坑难以定位的共同原因。

### 2.2 坑 A：开漏输出没有上拉能力

**现象**：程序烧进去，OLED 全黑，只有模块上的电源红灯亮着。逻辑分析仪（或示波器）量 SCL，发现它一直是低电平，从来没有回到过高。

**原因**：`GPIO_MODE_OUTPUT_OD` 在 STM32F1 的 HAL 里只配置了输出速度和开漏模式，**根本不读 `GPIO_Init->Pull` 字段**。`Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c` 里开漏分支的全部内容是：

```c
case GPIO_MODE_OUTPUT_OD:
  assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
  config = GPIO_Init->Speed + GPIO_CR_CNF_GP_OUTPUT_OD;
  break;
```

只有 `GPIO_MODE_INPUT` 和 EXTI 分支才会去看 `Pull`。所以写 `Pull = GPIO_PULLUP` 是白配——开漏输出本身不具备任何拉高能力，它只能把线拉低或「放手」，拉高必须靠外部电阻。如果 OLED 模块板上没焊 4.7k 上拉电阻，SCL 永远回不到高电平，一个时钟都发不出去。

**解法**：把每根线做成「双向」——写 1 时切成 `GPIO_MODE_INPUT` + `GPIO_PULLUP`（高阻释放，靠内部上拉把线拉高），写 0 时切成 `GPIO_MODE_OUTPUT_OD` + 写 `GPIO_PIN_RESET`（真正驱动低）。代码里封装成 `OLED_PinWrite(uint16_t pin, uint8_t level)`：

```c
static void OLED_PinWrite(uint16_t pin, uint8_t level)
{
	GPIO_InitTypeDef s = {0};
	s.Pin   = pin;
	s.Speed = GPIO_SPEED_FREQ_HIGH;

	if (level)
	{
		s.Mode = GPIO_MODE_INPUT;		/* 释放：高阻 + 内部上拉把线拉高 */
		s.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOB, &s);
	}
	else
	{
		s.Mode = GPIO_MODE_OUTPUT_OD;	/* 真正驱动低 */
		s.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &s);
		HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_RESET);
	}
}
```

标准库里的对应写法是 `GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;` 和 `GPIO_Mode_Out_OD;` 两种模式来回切，道理一样。

代价是每次翻转都要重写一次 CRL/CRH 寄存器，比纯输出慢，但在这个速率下无所谓。

### 2.3 坑 B：没有延时，SCL 频率失控

**现象**：屏幕能亮、能显示，但内容偶尔花屏、随机丢字，长杜邦线时更严重；示波器量 SCL 发现频率是几 MHz。

**原因**：原厂驱动一个 `NOP` 都没有。72MHz 下 GPIO 翻转的间隔只有几十纳秒，SCL 轻松跑到几 MHz，远超 SSD1306 的 400kHz 上限。再加上面包板接触电阻和杜邦线的寄生电容（几十 pF），边沿变缓，从机采样时刻对不上，就会出现随机错误。

**解法**：在每次引脚操作后加一个半周期延时，把 SCL 压到约 100kHz：

```c
{ volatile uint32_t d = 8; while (d--) { } }
```

`volatile` 不能省，否则编译器在 `-O2` 下会把这个空循环优化掉。按 72MHz、每次循环约 3~4 个周期估算，8 次约 0.3μs，配合函数调用开销，实测 SCL 在 100kHz 量级。这个值是试出来的，不是算出来的——真要靠算，得看反汇编数周期。

### 2.4 坑 C：引脚硬编码和实物不符

**现象**：代码编译通过、烧录成功、没有任何报错，屏幕全黑。

**原因**：驱动里用宏写死了引脚：

```c
#define OLED_W_SCL(x)	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, x)
#define OLED_W_SDA(x)	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, x)
```

如果实物接在 PB6/PB7，代码就在对着两根空脚翻转。因为 2.1 节说过 ACK 被丢弃，程序无从得知从机根本没收到数据，于是「编译通过、运行正常、屏幕不亮」。

**解法**：把引脚抽成宏，接哪改哪：

```c
#define OLED_SCL_PIN	GPIO_PIN_1
#define OLED_SDA_PIN	GPIO_PIN_0
```

这个坑在本项目上真发生过一次，排查花了很久，最后是用万用表量「代码里翻转的那个脚」和「OLED 模块 SCL 焊盘」是否导通才定位到。

**三个坑的共同教训**：软件 I²C 因为丢弃了 ACK，任何接线/电气问题都表现为同一个现象——黑屏。所以排查顺序应该是先量电平、再量通断、最后才怀疑代码。这一点和硬件 I²C 不同（见附录「细节 B」）。

---

## 三、硬件 I²C 的实现

### 3.1 CubeMX 配置

I2C1 外设参数（生成在 `Core/Src/i2c.c` 的 `MX_I2C1_Init()` 里）：

| 参数 | 值 | 说明 |
|---|---|---|
| Speed Mode | Standard | 标准模式 |
| Clock Speed | 100000 | 100kHz |
| Addressing Mode | 7-bit | SSD1306 用 7 位地址 |
| DutyCycle | `I2C_DUTYCYCLE_2` | 标准模式下无效，占位 |
| NoStretchMode | Disable | 允许从机拉低 SCL 做时钟延展 |

```c
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
```

引脚 PB6/PB7 由 `HAL_I2C_MspInit()` 配成 `GPIO_MODE_AF_OD`（复用开漏）：

```c
GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

CubeMX 顺带把 DMA1_Channel6（TX）/ Channel7（RX）也配好了，但本项目一帧只有 2 字节，用不上，保持未使用。

### 3.2 代码改法：只动两个函数

**改之前（软件版）**：`Core/Src/OLED.c` 里三个时序函数 + 两个宏，加两个写函数：

```c
#define OLED_W_SCL(x)	OLED_PinWrite(OLED_SCL_PIN, (uint8_t)(x))
#define OLED_W_SDA(x)	OLED_PinWrite(OLED_SDA_PIN, (uint8_t)(x))

void OLED_I2C_Start(void) { OLED_W_SDA(1); OLED_W_SCL(1); OLED_W_SDA(0); OLED_W_SCL(0); }
void OLED_I2C_Stop(void)  { OLED_W_SDA(0); OLED_W_SCL(1); OLED_W_SDA(1); }
void OLED_I2C_SendByte(uint8_t Byte) { /* for 循环 8 位 + 额外一个 ACK 时钟 */ }

void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);
	OLED_I2C_SendByte(0x00);
	OLED_I2C_SendByte(Command);
	OLED_I2C_Stop();
}
```

**改之后（硬件版）**：

```c
#include "i2c.h"					/* 取 hi2c1 句柄 */

#define OLED_I2C_ADDR		0x78	/* 0x3C << 1 */
#define OLED_I2C_TIMEOUT	10		/* ms，绝不用 HAL_MAX_DELAY */

void OLED_I2C_Init(void)
{
	/* 空实现：PB6/PB7 与 I2C1 时钟都由 MX_I2C1_Init() 负责 */
}

void OLED_WriteCommand(uint8_t Command)
{
	uint8_t buf[2];
	buf[0] = 0x00;		/* Co=0, D/C#=0 → 后面是命令 */
	buf[1] = Command;
	HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, buf, 2, OLED_I2C_TIMEOUT);
}

void OLED_WriteData(uint8_t Data)
{
	uint8_t buf[2];
	buf[0] = 0x40;		/* Co=0, D/C#=1 → 后面是数据 */
	buf[1] = Data;
	HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, buf, 2, OLED_I2C_TIMEOUT);
}
```

删掉的是 `OLED_I2C_Start()`、`OLED_I2C_Stop()`、`OLED_I2C_SendByte()` 三个函数、`OLED_PinWrite()` 及其延时、两个引脚宏。`OLED_I2C_Init()` 保留为空函数，只是为了不改动 `OLED_Init()` 的调用点。

**关键点：上层一行都没改。** `OLED_SetCursor` / `OLED_Clear` / `OLED_ShowChar` / `OLED_ShowString` / `OLED_ShowNum` 等函数只间接调用 `OLED_WriteCommand` / `OLED_WriteData`，所以换掉这两个函数就整体切到硬件 I²C。文件从 350 行缩到 286 行，减少的 64 行就是那一整套 bit-bang 时序。

这是这次移植最重要的一点：**接口稳定的驱动，替换底层实现不需要动上层**。如果当初把 `OLED_WriteCommand` 的实现直接散在 `OLED_ShowChar` 里，这次就得改十几处。

---

## 四、软硬件 I²C 对比

| 维度 | 软件 I²C | 硬件 I²C |
|---|---|---|
| 占用引脚 | 任意两个 GPIO（本项目 PB1/PB0） | 固定复用引脚（I²C1 = PB6/PB7） |
| CPU 开销 | 全程占用，每一位都要 CPU 打拍子 | 只有启动/收尾，传输由外设做 |
| 最高速率 | 受 CPU 速度与代码影响，本项约 100kHz | 100kHz / 400kHz（Fast Mode） |
| 时序精度 | 依赖代码，容易写错 | 由硬件波特率发生器保证 |
| 上拉电阻 | 可用「输入+内部上拉」绕开 | **必须外部上拉** |
| 多主机/仲裁 | 需要自己实现，基本不做 | 外设硬件支持 |
| 中断/DMA | 不支持 | 支持（本项目配了 DMA1_CH6/CH7，未使用） |
| 移植性 | 换芯片只需改两个宏 | 依赖具体外设与 CubeMX 配置 |
| 代码量 | 多（本项目 bit-bang 约 120 行） | 少（两个函数约 20 行） |
| 调试难度 | 逻辑分析仪一看就懂 | 出问题多为 BUSY 卡死，不好查 |

「必须外部上拉」这一条值得单独展开，见附录「细节 C」。

---

## 五、标准库写法对照

培训讲的是 HAL，但标准库（SPL）的写法可以对照记忆。软件 I²C 部分两套库完全一样（都是 GPIO 翻转，只是 `HAL_GPIO_WritePin` 换成 `GPIO_WriteBit`），差别全在硬件 I²C。

标准库没有 `HAL_I2C_Master_Transmit` 这种一把梭的封装，得手动走状态机：

```c
I2C_GenerateSTART(I2C1, ENABLE);
while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
I2C_Send7bitAddress(I2C1, 0x78, I2C_Direction_Transmitter);
while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
I2C_SendData(I2C1, 0x00);
while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
I2C_SendData(I2C1, Command);
while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
I2C_GenerateSTOP(I2C1, ENABLE);
```

HAL 的 `HAL_I2C_Master_Transmit` 就是把上面这一串（含所有事件轮询）包起来了。对应关系：

| 标准库 | HAL |
|---|---|
| `I2C_GenerateSTART` + 轮询 `MASTER_MODE_SELECT` | 函数内部完成 |
| `I2C_Send7bitAddress` + 轮询 `MASTER_TRANSMITTER_MODE_SELECTED` | 函数内部完成 |
| `I2C_SendData` + 轮询 `MASTER_BYTE_TRANSMITTED` | 函数内部循环 |
| `I2C_GenerateSTOP` | 函数内部完成 |
| 返回值靠自己判 `I2C_GetFlagStatus` | 统一返回 `HAL_StatusTypeDef` |

一个容易踩的点：标准库的 `I2C_Send7bitAddress(I2C1, 0x78, ...)` 里传的也是**左移过的 8 位形式**，和 HAL 一致——`0x3C << 1 = 0x78`。两套库在这一点上没区别，别以为标准库要传 0x3C。

---

## 六、编译结果与接线差异

### 6.1 编译结果

两版用同一套工具链（arm-none-eabi-gcc + CMake）编译，Debug 配置：

| 版本 | RAM | FLASH |
|---|---|---|
| 软件 I²C | 2568 B (12.54%) | 18528 B (28.27%) |
| 硬件 I²C | 2568 B (12.54%) | 19900 B (30.36%) |

硬件版 FLASH 多 1372 B，原因是链接了 `stm32f1xx_hal_i2c.c`——里面那一堆 `HAL_I2C_xxx` 函数、状态机、错误处理，即使只调用一个 `HAL_I2C_Master_Transmit` 也基本整份进来（链接器按函数段回收，但 `Master_Transmit` 依赖的 `I2C_WaitOnFlagUntilTimeout` 等一串都在）。

RAM 完全一样，2568 B。原因是两版的差异全在函数体内：`OLED_PinWrite` 里的 `GPIO_InitTypeDef` 和硬件版的 `buf[2]` 都是局部变量，活在栈上，不计入 `.data` / `.bss` 统计。而 `hi2c1` 这个句柄在两版工程里都存在——CubeMX 生成的 `i2c.c` 都编译进去了，软件版只是没调用它，所以句柄本身不构成差异。想让 RAM 也降下来，得把用不到的 HAL 模块从 CMake 的源文件列表里去掉。

### 6.2 接线差异

只动两根信号线，GND/VCC 不动：

| 信号 | 软件版 | 硬件版 |
|---|---|---|
| SCL / SCK | PB1 | **PB6** |
| SDA | PB0 | **PB7** |
| GND | 蓝负轨 | 不动 |
| VCC | 红正轨 3V3 | 不动 |

另外硬件版**必须**接 4.7k 上拉到 3V3（模块自带就不用）。判断模块是否自带：断电后用万用表蜂鸣档量模块的 `SCL 对 VCC`、`SDA 对 VCC`，响就是自带。

这一步最容易翻车的地方是**改完代码忘了换线**：PB1/PB0 上还插着杜邦线，PB6/PB7 空着，现象是全黑。软件版还有「引脚硬编码」这个坑叠加，硬件版至少 CubeMX 里配的就是 PB6/PB7，代码和配置是对齐的。

---

## 七、结论：什么时候用哪种

先排除一个错误答案：「硬件 I²C 更好」。两版在功能上完全等价，选择取决于约束。

**用硬件 I²C 的场合：**

- **引脚紧张**。I²C1 固定占用 PB6/PB7，但这两根脚本来也没别的复用价值；反过来说，如果板子上 PB6/PB7 已经被占用，硬件 I²C 就直接出局。
- **想省 CPU**。本项目一帧只有 2 字节，100kHz 下阻塞发送约 270μs（地址 + 控制字节 + 数据共 3 字节，每字节 9 位），刷新四行文字也就几十帧，实际感知不到。但如果要连续刷一整屏显存，差别就出来了：这个驱动是「每个数据字节一帧」，1024 字节就是 1024 帧 × 27 位 ≈ 27648 位，100kHz 下约 0.28 秒，全程 CPU 被占着；换成硬件 I²C + DMA 一次把整页推出去，CPU 只在开头结尾出现。
- **要高速率**。400kHz Fast Mode 只有硬件能做，软件 I²C 在 72MHz 上要跑到 400kHz 需要极紧的循环，实际很难稳定。
- **要多主机**。总线仲裁是纯硬件行为，软件实现基本不可行。

**用软件 I²C 的场合：**

- **引脚要留给别的外设**。这是本项目最现实的理由：PB6/PB7 如果被定时器或串口占了，软件 I²C 可以挪到任意两个空闲 GPIO 上，改两个宏就完事。
- **芯片的硬件 I²C 有勘误，不敢用**。STM32F1 的 I²C 外设是出了名的毛病多（见附录「细节 B」的 BUSY 卡死），在工业或长时间运行的场合，很多人宁可自己 bit-bang 图个可控。这也是本作业同时要求两版的原因之一——让你亲手体会这个取舍，而不是背结论。
- **只想快速验证一个传感器**。软件 I²C 不需要 CubeMX 配外设、不需要查复用表、不需要管 NVIC，复制一份驱动改两个引脚宏就能跑。
- **要单步调试时序**。软件 I²C 每一行代码对应一个电平动作，打断点就能看；硬件 I²C 的时序在片内，出了问题和代码对不上号。

**本项目的实际选择**：两版都交。必做项用软件版（PB1/PB0），选做项用硬件版（PB6/PB7）。硬件版代码更短、时序更可靠，但依赖外部上拉；软件版更灵活，但三个坑都得自己填。两版共用同一套上层显示代码，这个结构本身比「用哪个」更值得记住。

---

## 附：硬件 I²C 的三个必须知道的细节

### 细节 A：地址要传 `0x78` 不是 `0x3C`

SSD1306 的 7 位地址是 `0x3C`，但 `HAL_I2C_Master_Transmit` 的 `DevAddress` 参数要求传**左移一位后的 8 位形式**，即 `0x3C << 1 = 0x78`。最低位是读写方向位（0 = 写）。传 `0x3C` 会直接被 NACK。

函数原型（`Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_i2c.h:563`）：

```c
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                          uint8_t *pData, uint16_t Size, uint32_t Timeout);
```

顺带一提：软件版里 `OLED_I2C_SendByte(0x78)` 传的也是这个 8 位形式，因为软件版是自己一位一位发的，方向位得自己带上。两版一致，不是巧合。

### 细节 B：`Timeout` 绝不能给 `HAL_MAX_DELAY`

STM32F1 的硬件 I²C 有已知勘误：总线被从机拉死（比如上电时序不对、从机在复位过程中拽住 SDA）时，`SR2` 寄存器的 `BUSY` 位可能卡住不退，`HAL_I2C_Master_Transmit` 会永久阻塞在轮询里。

给 10ms 让它超时返回，至少主循环还能跑：

```c
#define OLED_I2C_TIMEOUT	10		/* ms */
```

更好的做法是检查返回值，超时就重新初始化 I2C 外设（`HAL_I2C_DeInit` + `HAL_I2C_Init`），本项目为了保持代码简洁没有加，只保证不卡死。

### 细节 C：`GPIO_MODE_AF_OD` 同样不读 `Pull` 字段

和坑 A 是同一个 HAL 分支逻辑：

```c
case GPIO_MODE_AF_OD:
  assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
  config = GPIO_Init->Speed + GPIO_CR_CNF_AF_OUTPUT_OD;
  break;
```

所以硬件 I²C **必须靠外部上拉电阻**（典型 4.7kΩ 接 3.3V），没有软件绕法。软件 I²C 还能用「输入 + 内部上拉」这个技巧绕过去（因为可以把引脚切成输入模式），硬件 I²C 不行——引脚必须处于复用开漏状态才能让外设接管，而这个状态下 Pull 字段被忽略。

这是硬件 I²C 相比软件 I²C 唯一「退步」的地方：它对硬件条件的要求更硬。内部上拉约 30~50kΩ，即使能配上也带不动 400kHz 的边沿，只适合 100kHz 以下的短线场合，所以正经设计还是得上外部 4.7k。
