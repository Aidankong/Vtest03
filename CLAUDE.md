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
- **ADC1**: Triggered by TIM2_CC2 at ~500 Hz, DMA1_Channel1 circular mode with interleaved 24-sample buffer (PA5/PA6/PA7; each channel 8 samples)
- **TIM2**: Output Compare CH2 for ADC triggering (PSC=71, ARR=999, CCR2=1, **OC2M=TOGGLE**); CC2E enabled via `HAL_TIM_OC_Start()`; OC2M must be TOGGLE (not TIMING) so OC2REF generates edges
- **TIM3**: 10 kHz PWM on PB0/CH3 and PB1/CH4 for power control (PSC=0, ARR=7199, PWM_DUTY_CUTOFF=7200 for 100% default)
- **I2C1**: SSD1306 OLED display on PB6/PB7 at 400 kHz
- **GPIO PA4** (label: POWER_CTRL): Digital output for power control

### Electrical Semantics (Critical - Do Not Change)

The external gate driver is **LOW-level active**:
- LOW = conduction (ON)
- HIGH = cutoff (OFF)

Default (safe) state after reset:
- PA4 = HIGH (OFF) — configured in IOC via `PA4.PinState=GPIO_PIN_SET`
- PWM (TIM3_CH3/PB0 and TIM3_CH4/PB1) = 100% duty (constant HIGH/OFF) - set as PWM_DUTY_CUTOFF = ARR + 1 = 7200

### Software Architecture Pattern

```
ISR/DMA Callbacks → Set Flags → Main Loop Processing → Output Updates
```

1. **DMA/ADC callbacks**: Only set data-ready flag, no business logic
2. **Main loop**: Calculate voltage, apply hysteresis, update outputs
3. **OLED updates**: 1 Hz in main loop only, never in ISRs

### Hysteresis Logic (Strict)

- Vbus > 51V: Cut off (PA4=HIGH, PWM=PWM_DUTY_CUTOFF/100%)
- Vbus < 48V: Conduct (PA4=LOW, PWM=PWM_DUTY_CONDUCTION/0%)
- 48V ≤ Vbus ≤ 51V: Maintain previous state (no toggling)

### Voltage Calculation

```
adc_avg = average of CH5 samples (8 samples from interleaved DMA frame)
Vadc = adc_avg * 3.3 / 4095
Vbus = Vadc * 25
```

### Software Filter (16-sample Moving Average)

The system uses a 16-sample moving average filter to eliminate ADC noise:

- **Buffer**: `filter_buffer[16]` - circular buffer for averaged samples
- **Counter**: `filter_count` - tracks number of samples filled (0-16)
- **Flag**: `filter_initialized` - set to 1 only after buffer is fully filled
- **Initialization**: During filling phase, averages only available samples; after 16 samples, uses full buffer

### PWM Duty Cycle Constants

PWM duty cycle is controlled via named constants (not hardcoded values):

```c
#define PWM_DUTY_CUTOFF     7200  // 100% duty = constant HIGH (ARR + 1)
#define PWM_DUTY_CONDUCTION 0     // 0% duty = constant LOW
```

Use HAL macro for setting PWM:
```c
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_DUTY_CUTOFF);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_DUTY_CUTOFF);
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
| ADC voltage 1 | PA5  | Analog    | ADC1_IN5, x25 divider (control)|
| ADC voltage 2 | PA6  | Analog    | ADC1_IN6                       |
| ADC voltage 3 | PA7  | Analog    | ADC1_IN7                       |
| PWM output 1  | PB0  | Output    | TIM3_CH3, 10 kHz               |
| PWM output 2  | PB1  | Output    | TIM3_CH4, 10 kHz               |
| OLED SCL      | PB6  | Output    | I2C1, 400 kHz                  |
| OLED SDA      | PB7  | I/O       | I2C1                           |

## STM32CubeMX Code Regeneration

After modifying `Vtest03.ioc` and regenerating code:
- Check that peripheral init code in `Core/Src/*.c` is updated
- User code between `/* USER CODE BEGIN */` markers is preserved
- Rebuild to verify toolchain integration

### IOC-First Safety Requirement

Do not rely on post-generation patches in `gpio.c` for PA4 safe level.
Keep safe default in IOC:
- `PA4.GPIOParameters` includes `PinState`
- `PA4.PinState=GPIO_PIN_SET`

### CubeMX Headless Regeneration

```bash
/Applications/STMicroelectronics/STM32CubeMX.app/Contents/MacOs/STM32CubeMX -q /Users/aidan/Documents/AI-system/Vtest03/cube_headless.txt
```
Script content (`cube_headless.txt`):
```
config load /Users/aidan/Documents/AI-system/Vtest03/Vtest03.ioc
project toolchain CMake
project name Vtest03
project generate
exit
```

For this repository, prefer `project generate`: `generate code /project_root` may generate `Src/Inc`, while the build consumes `Core/Src` and `Core/Inc`.

## Known Issues and Fixes

### TIM2 OC2 Mode Must Be TOGGLE, Not TIMING

**Symptom**: `DMA_EVT=0`, `DMA1_Ch1->CNDTR` frozen, ADC buffer static after one initial conversion.

**Root cause**: STM32CubeMX generates `TIM_OCMODE_TIMING` for TIM2_CH2. This is "frozen" mode — OC2REF is never driven. The ADC external trigger on STM32F1 is edge-sensitive on the **OC2REF signal**, not the CC2IF flag. With TIMING mode, OC2REF never toggles, so the ADC receives only one spurious edge at startup (from a pre-existing CC2IF condition) and then stops.

**Fix** (`Core/Src/tim.c`, USER CODE BEGIN TIM2_Init 2):
```c
/* OC2M is CCMR1[14:12]; TIM_OCMODE_TOGGLE=0x0030 is unshifted (OC1M position).
 * For CH2 it must be shifted left by 8 to reach the OC2M field. */
MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_OC2M, TIM_OCMODE_TOGGLE << 8U);
```

**Effect**: OC2REF flips on every CCR2 match (every 1 ms at ARR=999), producing a rising edge every 2 ms → ~500 Hz effective ADC trigger rate.

**Trigger chain**: TIM2 CCR2 match → OC2REF toggles → rising edge → ADC starts 3-channel scan (CH5/CH6/CH7) → 3 DMA transfers → repeat.

**After CubeMX regeneration**: This patch in `USER CODE BEGIN TIM2_Init 2` is preserved. Verify `TIM2_CCMR1` startup debug line shows `OC2M=3`.

---

### TIM2 Must Be Started with HAL_TIM_OC_Start, Not HAL_TIM_Base_Start

**Symptom**: CC2E bit not set, no CC2 events generated even with TOGGLE mode.

**Root cause**: `HAL_TIM_Base_Start()` only enables the counter. It does not set the CC2E bit in CCER, so the output compare channel is inactive.

**Fix** (`Core/Src/main.c`, USER CODE BEGIN 2):
```c
HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);  // enables CC2E
```

**Verify**: Startup debug should show `TIM2_CCER = 0x0010 (CC2E=1)`.

---

### TIM_OCMODE_TOGGLE Constant Is Not Pre-Shifted for CH2

**Symptom**: CCMR1 patch appears to succeed but `OC2M` reads back as 0 in debug output.

**Root cause**: `TIM_OCMODE_TOGGLE = 0x00000030` is the raw 3-bit mode value (`011`) placed at the OC**1**M bit position (bits [6:4]). The HAL's `TIM_OC2_SetConfig()` internally applies `OCMode << 8` to reach OC**2**M (bits [14:12]). When calling `MODIFY_REG` directly, the shift must be applied manually.

```c
// Wrong - value 0x0030 is masked to 0 by TIM_CCMR1_OC2M (0x7000):
MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_OC2M, TIM_OCMODE_TOGGLE);

// Correct - shift by 8 to align with OC2M field:
MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_OC2M, TIM_OCMODE_TOGGLE << 8U);
```

**General rule**: When directly writing timer OC mode bits for CH2/CH4, always shift the `TIM_OCMODE_*` constant left by 8 (CH2) or use the HAL `TIM_OC_InitTypeDef` path which handles the shift internally.

---

### Debugging ADC+DMA Pipeline: Register Checklist

When `DMA_EVT` stays at 0, check these registers in order:

| Register | Expected | Meaning if wrong |
|----------|----------|-----------------|
| `ADC1->CR2` | `DMA=1, EXTTRIG=1` | DMA or external trigger not enabled |
| `TIM2->CCER` | `CC2E=1 (0x0010)` | `HAL_TIM_Base_Start` used instead of `HAL_TIM_OC_Start` |
| `TIM2->CCMR1` | `OC2M=3 (0x3000)` | TOGGLE mode not applied or shift missing |
| `DMA1_Ch1->CCR` | `TCIE=1, HTIE=1` | DMA interrupts not enabled |
| `DMA1_Ch1->CNDTR` | Decrementing | If frozen: ADC not triggering DMA transfers |
| `DMA1->ISR` | Briefly non-zero | If always 0: DMA TC never fires (HAL clears flags in ISR) |
