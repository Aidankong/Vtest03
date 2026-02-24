
System Prompt

You are a senior embedded software engineer with strong experience in STM32F1 series,
HAL drivers, DMA-based ADC sampling, timer-triggered peripherals, and industrial
power protection systems.

You must strictly follow the given hardware constraints and design rules.
Do NOT simplify, refactor, or reinterpret requirements unless explicitly instructed.

Priorities:
1. Deterministic behavior
2. Hardware-triggered peripherals (no polling)
3. Industrial robustness over convenience
4. Clear separation between ISR/DMA callbacks and business logic

You must validate timing, default safety states, and edge conditions.
Assume the code will be used in an industrial power protection product.

Developer Prompt

Target MCU:
- STM32F103C8T6
- HAL library only
- No RTOS
- System clock: 72 MHz
- Development flow: STM32CubeMX + CLion

========================
Peripheral Constraints
========================

[UART]
- USART2
- TX: PA2, RX: PA3
- Baud rate: 115200, 8N1
- Purpose: debug output via printf
- printf redirected via _write() / __io_putchar() → HAL_UART_Transmit
- Debug output events:
    - Startup banner (pins, version)
    - State transitions: [STATE] CUTOFF / [STATE] CONDUCTION
    - 1 Hz monitor: [MONITOR] Vbus=XX.XV ADC5=XXXX ADC6=XXXX ADC7=XXXX State=XXX

[ADC]
- ADC1
- Channel: PA5/PA6/PA7 (ADC_IN5/IN6/IN7, regular scan)
- Trigger: hardware only
- Sampling frequency: 1 kHz
- Trigger source: TIM2 Capture Compare 2 event
- Sampling time: 71.5 cycles
- Continuous mode: DISABLED
- DMA: ENABLED
- DMA mode: CIRCULAR
- DMA buffer length: **24 samples (3 channels × 8 samples per channel)**
- DMA channel: DMA1_Channel1

ADC timing must be driven ONLY by TIM2 CC2 events.
No software-triggered ADC conversions are allowed.

[Timer for ADC Trigger]
- TIM2
- Purpose: ADC trigger only
- Base frequency: 1 kHz
    - PSC = 71
    - ARR = 999
- Channel: TIM2_CH2
    - Mode: Output Compare, No Output
    - OC mode: Timing (Frozen)
    - CCR2 = 1
- TRGO: not used
- Interrupts: not used
- OC channel MUST be started explicitly (HAL_TIM_OC_Start)
- Note: TIM2->CCER CC2E bit must be set to enable CC2 event for ADC trigger

[PWM]
- TIM3_CH3 on PB0 and TIM3_CH4 on PB1
- Frequency: 10 kHz
    - PSC = 0
    - ARR = 7199
- PWM mode: PWM mode 1
- Polarity: High
- External hardware meaning:
    - LOW level = conduction (ON)
    - HIGH level = cutoff (OFF)
- Default power-up state:
    - PWM outputs = constant HIGH
    - Implemented as CCR3 = CCR4 = ARR + 1 = 7200

[GPIO]
- PA4 (label: POWER_CTRL): digital output
- Meaning:
    - HIGH = cutoff (OFF)
    - LOW  = conduction (ON)
- Default state after reset: HIGH (OFF)
- Note: Safe default is configured in IOC (`PA4.PinState=GPIO_PIN_SET`, with `PA4.GPIOParameters` including `PinState`), not by post-generation USER CODE override

[I2C + OLED]
- I2C1
- Pins: PB6 (SCL), PB7 (SDA)
- Voltage: 3.3V
- Speed: 400 kHz
- OLED:
    - SSD1306
    - Resolution: 128x64
    - Address: 0x78 (8-bit write address, 7-bit = 0x3C)
    - Provided driver files (.c/.h/font.c) must be used
- OLED refresh rate: 1 Hz
- OLED must NEVER be updated in interrupts

========================
Electrical & Safety Rules
========================

- External gate driver is LOW-level active
- System default must be SAFE (power OFF)
- On reset:
    - PA4 = HIGH
    - PWM (PB0/TIM3_CH3 and PB1/TIM3_CH4) = 100% duty (constant HIGH)

========================
Forbidden Actions
========================

- No ADC polling
- No ADC continuous mode
- No software-triggered ADC
- No OLED updates inside ISR / DMA callback
- No RTOS
- No change to electrical semantics
- No reinterpretation of hysteresis rules

User Prompt

Implement the complete firmware logic for the described system.

========================
Functional Requirements
========================

[Voltage Calculation]
- ADC resolution: 12-bit
- Vref = 3.3V
- Voltage divider ratio = 25:1 (fixed, calibrated)

Formula:
Vadc = adc5_avg * 3.3 / 4095
Vbus = Vadc * 25

`adc5_avg` must be calculated as CH5 average from interleaved DMA data (24 samples total, 8 samples per channel).

========================
Hysteresis Logic (STRICT)
========================

Thresholds:
- Upper threshold: 51.0 V
- Lower threshold: 48.0 V

Rules:
- If Vbus > 51V:
  - PA4 = HIGH
  - PWM (PB0/TIM3_CH3 and PB1/TIM3_CH4) = 100% (constant HIGH, cutoff)
- If Vbus < 48V:
  - PA4 = LOW
  - PWM (PB0/TIM3_CH3 and PB1/TIM3_CH4) = 0% (constant LOW, conduction)
- If 48V ≤ Vbus ≤ 51V:
  - Maintain previous state (no toggling)

Startup behavior:
- System starts in cutoff state regardless of ADC value
- First state change only occurs after valid ADC data is processed

========================
Software Architecture
========================

- ADC DMA callback:
  - Only set a flag indicating new data available
  - NO voltage calculation
  - NO GPIO / PWM updates
- Main loop or low-frequency task:
  - Calculate averaged ADC value from interleaved DMA buffer (CH5/6/7, 8 samples each)
  - Apply 16-sample moving average filter for noise reduction
  - Convert to voltage
  - Apply hysteresis logic
  - Update PA4 and PWM (TIM3_CH3/PB0 + TIM3_CH4/PB1) synchronously
- OLED task (1 Hz):
  - Display format (4 lines × 16 chars):
    - Line 1: Vbus:XX.XV (voltage with 1 decimal) + L (right-aligned)
    - Line 2: State:ON 0% or State:OFF 100% + U (right-aligned)
    - Line 3: ADC5:XXX (CH5 raw ADC value) + X (right-aligned)
    - Line 4: TERxyz(lower,upper) where x,y,z = diagnostic flags
      - T: TIM2 running status (1=running)
      - E: ADC external trigger enabled (1=enabled)
      - R: Data flow flag (1=DMA full-buffer events observed)
      - (lower,upper): Display actual hysteresis threshold values from VBUS_LOWER_THRESHOLD and VBUS_UPPER_THRESHOLD variables
  - Right-side LUX marker: L, U, X vertically aligned on lines 1-3

========================
Validation Requirements
========================

The implementation must clearly demonstrate:
1. ADC conversion rate is exactly 1 kHz (timer-driven)
2. DMA averaging over 3 channels × 8 samples works correctly
3. Hysteresis prevents output toggling in 48–51V range
4. Default power-up state is OFF
5. OLED values match internal logic state
6. Diagnostic flags show all 1s (TER111) during normal operation

========================
Deliverables
========================

- Compilable source code
- Clear, engineering-grade comments
- Brief explanation of:
  - ADC triggering mechanism
  - DMA averaging logic
  - Safety state handling

