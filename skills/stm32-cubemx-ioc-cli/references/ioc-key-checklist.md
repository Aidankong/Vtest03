# IOC Key Checklist

Use this checklist whenever `.ioc` is modified.

## Identity and scope

- `Mcu.IP*`
- `Mcu.IPNb`
- `Mcu.Pin*`
- `Mcu.PinsNb`

## Pin binding keys

- `PAx/PBx/... .Signal`
- `PAx/PBx/... .Mode`
- `PAx/PBx/... .Locked`
- `PAx/PBx/... .GPIO_*` (Label/Mode/PuPd/Speed, when GPIO output/input is used)
- `PAx/PBx/... .PinState` (or legacy `GPIO_InitLevel`, depending on CubeMX IP schema)

## Peripheral configuration

- `USARTx.*` (baud, frame, virtual mode, DMA enable flags)
- `ADCx.*` (channel, rank, trigger, sampling time)
- `TIMx.*` (prescaler, period, channels, trigger output, compare/pwm settings)
- `I2Cx.*`, `SPIx.*` as needed

## DMA

- `Dma.Request*`
- `Dma.RequestsNb`
- `Dma.<REQUEST>.<index>.*` full channel settings

## Interrupts

- `NVIC.<IRQn>=...`
- Match IRQ with enabled peripheral/DMA channels

## Shared signal counters

- `SH.<signal>.ConfNb`
- `SH.<signal>.<n>=...`

## Init order

- `ProjectManager.functionlistsort`
- Ensure required `MX_*_Init` entries exist and order is valid (commonly GPIO/DMA before dependent peripherals).

## Post-generation verification

- Generated init files changed as expected:
  - `Core/Src/gpio.c`
  - `Core/Src/usart.c`
  - `Core/Src/adc.c`
  - `Core/Src/tim.c`
  - `Core/Src/dma.c`
  - `Core/Src/stm32f1xx_it.c` (or device equivalent)
- For low-active safety pins, verify generated preload level matches hardware-safe default before `HAL_GPIO_Init`.
- Build still passes.
