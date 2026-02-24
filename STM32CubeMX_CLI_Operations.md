# STM32CubeMX CLI 操作手册

## 目录

1. [环境说明](#环境说明)
2. [CLI 命令格式](#cli-命令格式)
3. [脚本文件格式](#脚本文件格式)
4. [工作流程](#工作流程)
5. [配置示例](#配置示例)
6. [常见问题](#常见问题)

---

## 环境说明

### 软件版本
- **STM32CubeMX**: 6.16.1
- **STM32Cube FW_F1**: V1.8.7
- **目标MCU**: STM32F103C8Tx
- **工具链**: CMake + GCC (arm-none-eabi-gcc)

### 项目路径
```
项目根目录: /Users/aidan/Documents/AI-system/Vtest03
IOC 文件: /Users/aidan/Documents/AI-system/Vtest03/Vtest03.ioc
脚本文件: /Users/aidan/Documents/AI-system/Vtest03/cube_headless.txt
```

---

## CLI 命令格式

### 三种运行模式

| 模式 | 参数 | 说明 |
|------|------|------|
| 交互模式 | `-i` | 显示 MX> 提示符，可输入命令 |
| 脚本模式 | `-s <script>` | 从脚本读取命令，需要 UI |
| 无头模式 | `-q <script>` | 从脚本读取命令，无 UI（推荐） |

### 执行命令

```bash
# 无头模式（推荐用于自动化）
/Applications/STMicroelectronics/STM32CubeMX.app/Contents/MacOs/STM32CubeMX -q /path/to/script.txt

# 脚本模式（需要图形环境）
/Applications/STMicroelectronics/STM32CubeMX.app/Contents/MacOs/STM32CubeMX -s /path/to/script.txt
```

---

## 脚本文件格式

### 基本脚本模板

创建文件 `cube_headless.txt`：

```
config load /Users/aidan/Documents/AI-system/Vtest03/Vtest03.ioc
project toolchain CMake
project name Vtest03
project generate
exit
```

### 关键命令说明

| 命令 | 说明 | 示例 |
|------|------|------|
| `config load <path>` | 加载 IOC 配置文件 | `config load /path/to/project.ioc` |
| `config save <path>` | 保存 IOC 配置文件 | `config save /path/to/project.ioc` |
| `generate code <path>` | 仅生成代码到指定路径 | `generate code /path/to/project_root` |
| `project generate` | 生成完整项目代码 | `project generate` |
| `project toolchain <name>` | 设置工具链 | `project toolchain CMake` |
| `project path <path>` | （可选）设置项目路径，部分版本可能返回 `KO` | `project path /home/user/project` |
| `project name <name>` | 设置项目名称 | `project name MyProject` |
| `exit` | 退出程序 | `exit` |

### 重要注意事项

1. **路径必须使用绝对路径**
2. **脚本文件每行一个命令**
3. **命令顺序**: 加载配置 → 生成项目 → 退出
4. **本项目优先 `project generate`**：`generate code` 可能生成到 `Src/Inc`，而构建使用 `Core/Src` 与 `Core/Inc`

---

## 工作流程

### 标准工作流程

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  1. 修改 IOC     │ ──▶ │  2. 运行 CLI    │ ──▶ │  3. 编译验证    │
│     配置文件     │     │     生成代码    │     │                 │
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

### 详细步骤

#### 步骤 1: 修改 IOC 文件

编辑 `Vtest03.ioc`，添加或修改配置。

**IOC 文件关键配置段：**

```
# 外设 IP 列表
Mcu.IP0=ADC1
Mcu.IP1=DMA
Mcu.IP2=NVIC
...
Mcu.IPNb=7

# 引脚列表
Mcu.Pin0=PA2
Mcu.Pin1=PA3
...
Mcu.PinsNb=11

# 外设配置（如 USART2）
USART2.BaudRate=115200
USART2.Dmaenabledrx=1
USART2.Dmaenabledtx=1
...

# DMA 配置
Dma.Request0=USART2_RX
Dma.Request1=USART2_TX
Dma.RequestsNb=2
...

# NVIC 中断配置
NVIC.USART2_IRQn=true\:0\:0\:false\:false\:true\:true\:true\:true
...

# 初始化函数列表
ProjectManager.functionlistsort=1-SystemClock_Config-RCC-false-HAL-false,2-MX_GPIO_Init-GPIO-false-HAL-true,...
```

#### 步骤 2: 运行 CLI 生成代码

```bash
/Applications/STMicroelectronics/STM32CubeMX.app/Contents/MacOs/STM32CubeMX -q /Users/aidan/Documents/AI-system/Vtest03/cube_headless.txt
```

**成功输出标志：**
```
[INFO] CodeEngine:291 - Generated code: .../Core/Src/main.c
...
OK
exit
Bye bye
```

#### 步骤 3: 编译验证

```bash
rm -rf build/Debug
cmake --preset Debug
cmake --build build/Debug
```

---

## 配置示例

### 示例 1: 添加 USART2 异步串口

**IOC 文件修改：**

```ini
# 添加 IP
Mcu.IP6=USART2
Mcu.IPNb=7

# 添加引脚
Mcu.Pin0=PA2
Mcu.Pin1=PA3
Mcu.PinsNb=11

# PA2/PA3 配置
PA2.Locked=true
PA2.Mode=Asynchronous
PA2.Signal=USART2_TX
PA3.Locked=true
PA3.Mode=Asynchronous
PA3.Signal=USART2_RX

# USART2 参数
USART2.BaudRate=115200
USART2.WordLength=WORDLENGTH_8B
USART2.Parity=PARITY_NONE
USART2.StopBits=STOPBITS_1
USART2.VirtualMode=VM_ASYNC

# 更新 functionlistsort
ProjectManager.functionlistsort=...,6-MX_USART2_UART_Init-USART2-false-HAL-true
```

### 示例 2: 添加 DMA 支持

**IOC 文件修改：**

```ini
# 添加 DMA IP
Mcu.IP1=DMA

# DMA 请求配置
Dma.Request0=USART2_RX
Dma.Request1=USART2_TX
Dma.RequestsNb=2

# DMA RX 通道配置
Dma.USART2_RX.0.Direction=DMA_PERIPH_TO_MEMORY
Dma.USART2_RX.0.Instance=DMA1_Channel6
Dma.USART2_RX.0.MemDataAlignment=DMA_MDATAALIGN_BYTE
Dma.USART2_RX.0.MemInc=DMA_MINC_ENABLE
Dma.USART2_RX.0.Mode=DMA_NORMAL
Dma.USART2_RX.0.PeriphDataAlignment=DMA_PDATAALIGN_BYTE
Dma.USART2_RX.0.PeriphInc=DMA_PINC_DISABLE
Dma.USART2_RX.0.Priority=DMA_PRIORITY_LOW

# DMA TX 通道配置
Dma.USART2_TX.1.Direction=DMA_MEMORY_TO_PERIPH
Dma.USART2_TX.1.Instance=DMA1_Channel7
...

# USART2 启用 DMA
USART2.Dmaenabledrx=1
USART2.Dmaenabledtx=1

# 添加 MX_DMA_Init
ProjectManager.functionlistsort=...,3-MX_DMA_Init-DMA-false-HAL-true,...
```

### 示例 3: 添加中断

**IOC 文件修改：**

```ini
# USART2 中断
NVIC.USART2_IRQn=true\:0\:0\:false\:false\:true\:true\:true\:true

# DMA 中断
NVIC.DMA1_Channel6_IRQn=true\:0\:0\:false\:false\:true\:false\:true\:true
NVIC.DMA1_Channel7_IRQn=true\:0\:0\:false\:false\:true\:false\:true\:true
```

**中断配置参数格式：**
```
NVIC.<IRQn>=<enabled>\:<preemptionPriority>\:<subPriority>\:<enabled?>
```

### 示例 4: 添加 ADC 通道

**IOC 文件修改：**

```ini
# 添加 ADC1 IP
Mcu.IP0=ADC1

# ADC1 配置
ADC1.Channel-1\#ChannelRegularConversion=ADC_CHANNEL_5
ADC1.Rank-1\#ChannelRegularConversion=1
ADC1.SamplingTime-1\#ChannelRegularConversion=ADC_SAMPLETIME_1CYCLE_5
ADC1.NbrOfConversionFlag=1
ADC1.master=1

# PA5 引脚配置
PA5.GPIO_Label=VBUS_ADC
PA5.Locked=true
PA5.Signal=ADCx_IN5

# 共享信号配置
SH.ADCx_IN5.0=ADC1_IN5,IN5
SH.ADCx_IN5.ConfNb=1
```

---

## STM32F103C8T6 资源映射

### USART 外设

| 外设 | TX | RX | DMA TX | DMA RX |
|------|-----|-----|--------|--------|
| USART1 | PA9 | PA10 | DMA1_Ch4 | DMA1_Ch5 |
| USART2 | PA2 | PA3 | DMA1_Ch7 | DMA1_Ch6 |
| USART3 | PB10 | PB11 | DMA1_Ch2 | DMA1_Ch3 |

### ADC 通道

| 通道 | 引脚 |
|------|------|
| ADC1_IN0 | PA0 |
| ADC1_IN1 | PA1 |
| ADC1_IN2 | PA2 |
| ADC1_IN3 | PA3 |
| ADC1_IN4 | PA4 |
| ADC1_IN5 | PA5 |
| ADC1_IN6 | PA6 |
| ADC1_IN7 | PA7 |
| ADC1_IN8 | PB0 |
| ADC1_IN9 | PB1 |

### TIM 通道

| 定时器 | CH1 | CH2 | CH3 | CH4 |
|--------|-----|-----|-----|-----|
| TIM1 | PA8 | PA9 | PA10 | PA11 |
| TIM2 | PA0/PA5/PA15 | PA1/PB3 | PA2 | PA3 |
| TIM3 | PA6/PB4/PC6 | PA7/PB5/PC7 | PB0/PC8 | PB1/PC9 |
| TIM4 | PB6/PD12 | PB7/PD13 | PB8/PD14 | PB9/PD15 |

---

## 常见问题

### Q1: CLI 脚本执行无效果

**原因**: 脚本文件路径需要使用绝对路径

**解决方案**:
```bash
# 错误
./STM32CubeMX -q script.txt

# 正确
./STM32CubeMX -q /home/user/project/script.txt
```

### Q2: 生成的代码缺少初始化函数

**原因**: `ProjectManager.functionlistsort` 未包含对应的初始化函数

**解决方案**:
添加初始化函数到列表：
```ini
ProjectManager.functionlistsort=...,N-MX_XXX_Init-XXX-false-HAL-true
```

### Q3: 外设未生成代码

**原因**:
1. IP 未添加到 `Mcu.IPx` 列表
2. `Mcu.IPNb` 数量不正确
3. 引脚未正确配置

**解决方案**:
检查 IOC 文件中的：
- `Mcu.IP0=XXX`
- `Mcu.IPNb=N`
- `PAx.Signal=XXX`

### Q4: DMA 未关联到外设

**原因**: 外设的 DMA 启用参数未设置

**解决方案**:
```ini
# 对于 USART
USART2.Dmaenabledrx=1
USART2.Dmaenabledtx=1
```

### Q5: 中断未生效

**原因**: NVIC 配置未添加或格式错误

**解决方案**:
```ini
NVIC.USART2_IRQn=true\:0\:0\:false\:false\:true\:true\:true\:true
```

### Q6: IOC 改了但运行行为没变

**原因**: 使用 `generate code` 后代码生成到了 `Src/`、`Inc/`，但工程编译的是 `Core/Src`、`Core/Inc`

**解决方案**:
1. 脚本改为 `project generate`
2. 重新运行 CLI 并确认日志出现 `Generated code: .../Core/Src/...`
3. 再执行 `cmake --preset Debug && cmake --build build/Debug`

---

## 快速参考命令

```bash
# 生成代码
/Applications/STMicroelectronics/STM32CubeMX.app/Contents/MacOs/STM32CubeMX -q /Users/aidan/Documents/AI-system/Vtest03/cube_headless.txt

# 编译项目
cmake --preset Debug && cmake --build build/Debug

# 清理重建
rm -rf build/Debug && cmake --preset Debug && cmake --build build/Debug

# 检查生成的文件
ls -la Core/Src/
ls -la Core/Inc/

# 检查编译结果
arm-none-eabi-size build/Debug/Vtest03.elf
```

---

## 版本历史

| 日期 | 版本 | 说明 |
|------|------|------|
| 2026-02-24 | 1.1 | 更新为 Vtest03 实际绝对路径；补充 `generate code` 目录错位风险与 `project generate` 规避策略 |
| 2026-02-24 | 1.0 | 初始版本，包含 USART2+DMA+中断 配置示例 |

---

## 参考资料

- [STM32CubeMX User Manual (UM1718)](https://www.st.com/resource/en/user_manual/um1718-stm32cubemx-description-stmicroelectronics.pdf)
- [STM32F103x8 Datasheet](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)
- [STM32F1 HAL Driver Description](https://www.st.com/resource/en/user_manual/um1785-description-of-stm32f1-hal-and-lowlayer-drivers-stmicroelectronics.pdf)
