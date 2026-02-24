# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an industrial power protection system firmware for STM32F103C8T6. The system monitors bus voltage (48V-51V range with hysteresis) and controls power switching via PWM and GPIO. The design prioritizes deterministic behavior and hardware-driven timing over software convenience.

## Build Commands

```bash
# Configure and build (using CMake presets)
cmake --preset Debug                    # Configure debug build
cmake --build build/Debug               # Build debug

# Other build types available: Release, RelWithDebInfo, MinSizeRel
cmake --preset Release
cmake --build build/Release
```

The project uses CMake + Ninja with ARM GCC toolchain. Build outputs go to `build/<preset>/`.

## Development Workflow

This is an STM32CubeMX + CLion project. The `Vtest03.ioc` file is the STM32CubeMX configuration. When regenerating code via STM32CubeMX, user code between `/* USER CODE BEGIN */` and `/* USER CODE END */` markers is preserved.

## Architecture

### Hardware-Driven Peripherals (No Polling)

The system uses hardware-triggered peripherals exclusively:

- **USART2**: Debug output on PA2(TX)/PA3(RX) at 115200 baud; printf redirected via `_write()`
- **ADC1**: Triggered by TIM2_CC2 at 1 kHz, DMA1_Channel1 circular mode with 8-sample buffer; input on PA5 (ADC_IN5)
- **TIM2**: Output Compare CH2 for ADC triggering (PSC=71, ARR=999, CCR2=1); TIM2->CCER CC2E must be set manually
- **TIM3**: 10 kHz PWM on PB0/CH3 for power control (PSC=0, ARR=7199, CCR3=7200 for 100% default)
- **I2C1**: SSD1306 OLED display on PB6/PB7 at 400 kHz
- **GPIO PA4** (label: POWER_CTRL): Digital output for power control

### Electrical Semantics (Critical - Do Not Change)

The external gate driver is **LOW-level active**:
- LOW = conduction (ON)
- HIGH = cutoff (OFF)

Default (safe) state after reset:
- PA4 = HIGH (OFF) — CubeMX generates RESET; corrected in `MX_GPIO_Init` USER CODE block
- PWM (TIM3_CH3/PB0) = 100% duty (constant HIGH/OFF) - set as CCR3 = ARR + 1 = 7200

### Software Architecture Pattern

```
ISR/DMA Callbacks → Set Flags → Main Loop Processing → Output Updates
```

1. **DMA/ADC callbacks**: Only set data-ready flag, no business logic
2. **Main loop**: Calculate voltage, apply hysteresis, update outputs
3. **OLED updates**: 1 Hz in main loop only, never in ISRs

### Hysteresis Logic (Strict)

- Vbus > 51V: Cut off (PA4=HIGH, TIM3_CH3 CCR3=7200/100%)
- Vbus < 48V: Conduct (PA4=LOW, TIM3_CH3 CCR3=0/0%)
- 48V ≤ Vbus ≤ 51V: Maintain previous state (no toggling)

### Voltage Calculation

```
adc_avg = average of 8 DMA samples
Vadc = adc_avg * 3.3 / 4095
Vbus = Vadc * 25
```

## Key Constraints from PRD.md

- **No ADC polling, continuous mode, or software triggering**
- **No OLED updates in interrupts/DMA callbacks**
- **No RTOS**
- **Deterministic timing over convenience**
- **All peripherals must be interrupt/DMA-driven**

## File Structure

- `Core/Src/`, `Core/Inc/`: Main application code (STM32CubeMX generated)
- `Core/Src/main.c`: Main application entry point
- `Core/Src/adc.c`, `dma.c`, `tim.c`, `i2c.c`, `gpio.c`, `usart.c`: Peripheral initialization
- `Core/Inc/usart.h`: USART2 handle declaration (`extern UART_HandleTypeDef huart2`)
- `Core/Src/oled.c`, `Core/Inc/oled.h`: SSD1306 driver (do not modify)
- `Core/Inc/oled_font.h`: OLED font data (filename is lowercase; `oled.c` must use `#include "oled_font.h"`)
- `Drivers/`: STM32 HAL and CMSIS (STM32CubeMX generated)
- `cmake/gcc-arm-none-eabi.cmake`: ARM toolchain configuration
- `cmake/stm32cubemx/`: Generated STM32CubeMX CMake files
- `PRD.md`: Product Requirements Document - authoritative specification

## Pin Assignment Summary

| Function      | Pin  | Direction | Notes                          |
|---------------|------|-----------|--------------------------------|
| UART TX       | PA2  | Output    | USART2, 115200 baud            |
| UART RX       | PA3  | Input     | USART2                         |
| Power control | PA4  | Output    | HIGH=OFF, LOW=ON (LOW-active)  |
| ADC voltage   | PA5  | Analog    | ADC1_IN5, x25 divider          |
| PWM output    | PB0  | Output    | TIM3_CH3, 10 kHz               |
| OLED SCL      | PB6  | Output    | I2C1, 400 kHz                  |
| OLED SDA      | PB7  | I/O       | I2C1                           |

## STM32CubeMX Code Regeneration

After modifying `Vtest03.ioc` and regenerating code:
- Check that peripheral init code in `Core/Src/*.c` is updated
- User code between `/* USER CODE BEGIN */` markers is preserved
- Rebuild to verify toolchain integration

### Known Post-Generation Fix Required

CubeMX generates PA4 initial state as `GPIO_PIN_RESET` (LOW). This is **unsafe**.
The override is placed in `gpio.c` inside `/* USER CODE BEGIN MX_GPIO_Init_2 */`:
```c
HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_SET);
```
Do not remove this block when regenerating code.

### CubeMX Headless Regeneration

```bash
/home/luxsan_aard/STM32CubeMX/STM32CubeMX -q /path/to/cube_headless.txt
```
Script content (`cube_headless.txt`):
```
config load /home/luxsan_aard/workspace/STM32/Vtest03/Vtest03/Vtest03.ioc
project generate
exit
```
