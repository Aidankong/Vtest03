# AGENTS.md - AI编码助手项目指南

本文档为AI编码助手提供关于Vtest03项目的必要背景信息，帮助你理解和修改代码。

## 项目概述

**Vtest03** 是一个基于STM32F103C8T6的工业电源保护系统固件。系统监测总线电压（48V-51V范围，带迟滞），通过PWM和GPIO控制功率切换。设计优先考虑确定性行为和硬件驱动的时序，而非软件便利性。

### 核心特性

- **UART调试输出**：USART2（PA2/PA3），115200波特率，printf重定向，实时状态上报
- **硬件触发的ADC采样**：TIM2_CC2以1kHz触发ADC，3通道采样（PA5/PA6/PA7），DMA循环模式传输
- **软件滤波器**：16样本滑动平均，消除ADC噪声和悬空波动
- **迟滞电压保护**：48V以下导通，51V以上关断，48-51V之间维持状态
- **三路控制输出**：PA4（GPIO）+ TIM3_CH3/PB0 + TIM3_CH4/PB1（PWM），实现冗余控制
- **实时OLED显示**：电压、状态、ADC原始值、诊断信息
- **零RTOS设计**：纯中断+主循环架构，确保确定性

## 技术栈

| 组件 | 规格 |
|------|------|
| MCU | STM32F103C8T6 (Cortex-M3, 72MHz) |
| HAL库 | STM32F1xx HAL Driver |
| 固件包 | STM32Cube FW_F1 V1.8.7 |
| CubeMX版本 | 6.16.1 |
| 构建系统 | CMake + Ninja |
| 工具链 | ARM GCC (arm-none-eabi-gcc) |
| RTOS | 无 |

## 项目结构

```
Vtest03/
├── Core/
│   ├── Src/                    # 应用程序源代码
│   │   ├── main.c              # 主应用逻辑（电压计算、迟滞控制）
│   │   ├── adc.c               # ADC配置（TIM2_CC2外部触发）
│   │   ├── dma.c               # DMA配置（循环模式）
│   │   ├── tim.c               # TIM2(触发源)/TIM3(PWM)配置
│   │   ├── gpio.c              # GPIO配置（PA4控制输出）
│   │   ├── i2c.c               # I2C配置（OLED）
│   │   ├── usart.c             # USART2配置（调试输出）
│   │   ├── oled.c              # SSD1306驱动（自定义）
│   │   ├── stm32f1xx_it.c      # 中断服务程序
│   │   └── ...                 # 其他HAL相关文件
│   └── Inc/                    # 头文件
│       ├── main.h
│       ├── usart.h             # USART2句柄声明
│       ├── oled.h
│       ├── oled_font.h         # 字体数据（注意：小写文件名）
│       └── ...
├── Drivers/                    # STM32 HAL和CMSIS（CubeMX生成）
│   ├── CMSIS/
│   └── STM32F1xx_HAL_Driver/
├── cmake/
│   ├── gcc-arm-none-eabi.cmake # ARM工具链配置
│   └── stm32cubemx/
│       └── CMakeLists.txt      # CubeMX生成的CMake配置
├── build/                      # 构建输出目录
├── Vtest03.ioc                 # STM32CubeMX配置文件（单一真源）
├── cube_headless.txt           # CubeMX无头模式脚本
├── CMakeLists.txt              # 主CMake配置
├── CMakePresets.json           # CMake预设配置
├── startup_stm32f103xb.s       # 启动汇编文件
└── *.ld                        # 链接器脚本
```

## 硬件架构

### 引脚定义

| 功能 | 引脚 | 模式 | 说明 |
|------|------|------|------|
| UART TX | PA2 | 复用推挽 | USART2发送，115200波特率 |
| UART RX | PA3 | 输入 | USART2接收 |
| 控制输出 | PA4 | 推挽输出 | **低电平有效**：LOW=导通，HIGH=关断 |
| ADC输入1 | PA5 | 模拟输入 | 主控制通道（ADC1_IN5） |
| ADC输入2 | PA6 | 模拟输入 | 扩展采样通道（ADC1_IN6） |
| ADC输入3 | PA7 | 模拟输入 | 扩展采样通道（ADC1_IN7） |
| PWM输出1 | PB0 | 复用推挽 | TIM3_CH3，10kHz |
| PWM输出2 | PB1 | 复用推挽 | TIM3_CH4，10kHz |
| I2C SCL | PB6 | 开漏输出 | OLED时钟线 |
| I2C SDA | PB7 | 开漏输出 | OLED数据线 |

### 电气语义（关键 - 严禁修改）

外部门驱动为**低电平有效**：
- **LOW** = 导通（ON）
- **HIGH** = 关断（OFF）

**安全默认状态**（复位后）：
- PA4 = HIGH（OFF）— 在IOC中配置 `PA4.PinState=GPIO_PIN_SET`
- PWM（PB0/PB1）= 100%占空比（CCR3/CCR4=7200，恒定HIGH/OFF）

## 软件架构

### 架构模式

```
ISR/DMA回调 → 设置标志 → 主循环处理 → 输出更新
```

1. **DMA/ADC回调**：仅设置数据就绪标志，不执行业务逻辑
2. **主循环**：计算电压、应用迟滞、更新输出
3. **OLED更新**：仅在主循环中1Hz更新，禁止在ISR中调用

### 数据流

```
TIM2 CC2 (1kHz) → ADC转换 → DMA传输 → 设置标志 → 主循环处理
```

### 关键模块

#### 1. ADC采样系统
- **触发源**：TIM2_CC2，1kHz
- **DMA缓冲区**：24样本（3通道×每通道8样本）
- **采样时间**：71.5周期
- **布局**：交错存储 [CH5,CH6,CH7, CH5,CH6,CH7, ...]

#### 2. 软件滤波器（16样本滑动平均）
- **缓冲区**：`filter_buffer[16]` 
- **计数器**：`filter_count` - 跟踪已填充样本数
- **初始化**：缓冲区完全填满后才标记初始化完成
- **延迟**：16ms

#### 3. 迟滞控制逻辑

```c
#define VBUS_UPPER_THRESHOLD 51.0f  // 上限阈值
#define VBUS_LOWER_THRESHOLD 48.0f  // 下限阈值
```

- **Vbus > 51V**：关断（PA4=HIGH, PWM=100%）
- **Vbus < 48V**：导通（PA4=LOW, PWM=0%）
- **48V ≤ Vbus ≤ 51V**：维持当前状态（防抖）

#### 4. PWM占空比常量

```c
#define PWM_DUTY_CUTOFF     7200  // 100% = ARR + 1 = 恒定HIGH
#define PWM_DUTY_CONDUCTION 0     // 0% = 恒定LOW
```

使用HAL宏设置：
```c
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_DUTY_CUTOFF);
```

## 构建命令

### 环境要求

- **IDE**: CLion（推荐）或 VS Code
- **工具链**: ARM GCC（STM32CubeCLT包含）
- **调试器**: ST-Link V2/V3

### 命令行构建

```bash
# 配置Debug构建
cmake --preset Debug

# 构建
cmake --build build/Debug

# 其他构建类型
cmake --preset Release
cmake --preset RelWithDebInfo
cmake --preset MinSizeRel

# 清理重建
rm -rf build/Debug
cmake --preset Debug
cmake --build build/Debug
```

### 构建输出

- `build/Debug/Vtest03.elf` - 可执行文件（带调试符号）
- `build/Debug/Vtest03.map` - 内存映射文件
- `build/Debug/Vtest03.bin` - 二进制固件

## STM32CubeMX代码再生

### 修改IOC后的标准流程

```
修改 Vtest03.ioc → 运行CLI生成代码 → 编译验证
```

### CLI无头模式（推荐）

```bash
/Applications/STMicroelectronics/STM32CubeMX.app/Contents/MacOs/STM32CubeMX -q /path/to/cube_headless.txt
```

脚本内容 (`cube_headless.txt`)：
```
config load /absolute/path/to/Vtest03.ioc
project toolchain CMake
project name Vtest03
project generate
exit
```

### 重要注意事项

1. **使用 `project generate`**：`generate code <path>` 可能生成到 `Src/Inc`，而构建使用 `Core/Src`、`Core/Inc`
2. **路径必须使用绝对路径**
3. **USER CODE保留**：CubeMX会保留 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 之间的代码

## 代码规范

### 禁止事项（严格遵守）

- ❌ **禁止ADC轮询** - 必须使用TIM2_CC2硬件触发
- ❌ **禁止ADC连续模式** - 单次转换+外部触发
- ❌ **禁止软件触发ADC** - 仅硬件触发
- ❌ **禁止在ISR/DMA回调中更新OLED** - 必须在主循环
- ❌ **禁止使用RTOS** - 纯中断+主循环架构
- ❌ **禁止修改电气语义** - LOW=导通，HIGH=关断
- ❌ **禁止重新定义迟滞规则** - 48V/51V阈值严格固定

### 推荐实践

- ✅ **ISR/DMA回调只设置标志**，不执行业务逻辑
- ✅ **主循环执行所有计算和输出更新**
- ✅ **OLED更新严格在主循环**，1Hz频率
- ✅ **确定性优先于便利性**
- ✅ **所有外设必须中断/DMA驱动**

### 文件命名规范

- 用户代码放在 `Core/Src/` 和 `Core/Inc/`
- 自定义驱动（如OLED）放在用户代码区域
- 注意：`oled_font.h` 是小写，引用时必须使用 `"oled_font.h"`

## 调试指南

### 串口调试输出

波特率115200-8N1，连接PA2(TX)/PA3(RX)：

```
========================================
Power Protection System v2.0
ADC: PA5/PA6/PA7 (CH5/6/7), PWM: PB0/CH3 + PB1/CH4, GPIO: PA4
UART: PA2/PA3 @ 115200
========================================
[STATE] CUTOFF
[MONITOR] Vbus=52.3V ADC5=516 ADC6=512 ADC7=509 State=CUTOFF
```

### OLED诊断信息

第4行格式：`TERxyz(下限,上限)`

| 标志 | 含义 | 正常值 |
|------|------|--------|
| T | TIM2运行状态 | 1 |
| E | ADC外部触发使能 | 1 |
| R | 数据流心跳 | 1 |

**正常工作时**：`TER111(48,51)`

### 常见问题排查

| 症状 | 可能原因 | 解决 |
|------|----------|------|
| `TER011` (T=0) | TIM2未启动 | 检查 `HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2)` |
| `TER101` (E=0) | ADC外部触发未配置 | 检查 `adc.c` 外部触发设置 |
| `TER110` (R=0) | DMA中断未触发 | 检查DMA中断使能标志 |
| ADC读数乱跳 | 引脚悬空/干扰 | 软件滤波器已启用，正常 |
| Vbus显示不完整 | 浮点格式化问题 | 已改用整数运算 |

## 烧录固件

### 使用ST-Link Utility

1. 连接ST-Link到STM32F103C8T6
2. 打开ST-Link Utility
3. File → Open File → 选择 `build/Debug/Vtest03.elf`
4. Target → Program & Verify
5. 点击"Start"

### 使用CLion调试器

1. 配置Run/Debug Configuration为"OpenOCD Download & Run"
2. 点击"Debug"按钮
3. 程序自动烧录并暂停在main()

## 内存占用（参考）

```
FLASH: ~37 KB / 64 KB (57%)
RAM:   ~2 KB / 20 KB (10%)
```

## 参考文档

- `README.md` - 详细硬件接线说明、工作原理、修改记录
- `PRD.md` - 产品需求文档（技术规格权威来源）
- `CLAUDE.md` - Claude Code项目指南
- `STM32CubeMX_CLI_Operations.md` - CubeMX CLI操作手册

## 版本信息

- **固件版本**: v2.1
- **最后更新**: 2026-02-24
- **CubeMX版本**: 6.16.1
- **HAL库版本**: STM32Cube FW_F1 V1.8.7
