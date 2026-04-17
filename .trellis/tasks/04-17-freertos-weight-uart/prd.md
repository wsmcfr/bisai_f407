# FreeRTOS Weight UART

## Goal
Implement a small STM32F407 firmware feature that reads a 5kg HX711-based scale under FreeRTOS and returns the measured weight over UART on command.

## Requirements
- Keep all hand-written user code under `User/`.
- Put application-layer code under `User/App/`.
- Put driver-layer code under `User/Driver/`.
- Reuse the existing CubeMX-generated HAL + FreeRTOS + USART1 skeleton.
- Create a dedicated HX711 driver module.
- Create a FreeRTOS task that continuously updates the latest scale reading.
- Use DMA + UART idle-line reception for serial command input.
- Only send the current weight when a PC command is received; do not send weight periodically by default.
- Keep generated `Core/` files limited to integration glue only.

## Acceptance Criteria
- [ ] `User/App/` and `User/Driver/` contain the new hand-written modules.
- [ ] The firmware includes an HX711 driver that can read raw conversion data.
- [ ] A FreeRTOS task acquires and updates the latest weight value.
- [ ] `USART1` receives commands through DMA + idle detection.
- [ ] The firmware sends human-readable weight information only after a valid query command.
- [ ] The implementation is documented with Chinese comments in user code.

## Technical Notes
- Platform: STM32F407ZGTx, HAL, CMSIS-RTOS2 over FreeRTOS.
- Existing serial port: `USART1`, 115200 baud, TX/RX enabled.
- Existing RTOS bootstrap: `Core/Src/freertos.c` with `defaultTask`.
- HX711 GPIO pin mapping is not defined in the repository yet and may need user confirmation before finalizing hardware glue.
- Planned minimum serial command set: `GET` returns the latest weight value.
