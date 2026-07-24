# Task5b_PID — Autonomous Line-Following & Color-Sorting Robot

An STM32F103RB (Nucleo-64) based autonomous robot that follows a line through a warehouse-style arena, picks up objects using an electromagnet, identifies their color with a TCS3200-style color sensor, and sorts them into matching colored bins. Built with STM32CubeIDE / STM32 HAL.

![Arena Layout](arena.jpg)
*Example arena: objects are picked up at the colored junctions in the bottom "farm" zone and delivered to the matching colored bins in the top depot zone.*

## Overview

The robot runs a closed loop of:

1. **Line following** using a 5-channel IR sensor array and a PID controller.
2. **Object pickup** via an electromagnet, triggered by a dedicated object-detect sensor.
3. **Color classification** of the picked-up object using a TCS3200-style color sensor (frequency-based RGB read via timer input capture).
4. **Junction counting** on the return path to determine when the robot has reached the correct colored bin.
5. **Drop-off** by de-energizing the electromagnet, then resuming the line-following loop.

After two objects have been picked up and sorted, the robot stops and flashes a white status LED to signal completion.

## Hardware

| Component | Function |
|---|---|
| STM32F103RB (Nucleo-64) | Main MCU |
| 5x IR reflectance sensors | Line position sensing (ADC1, DMA) |
| 2x DC motors + H-bridge driver | Differential drive (TIM2 PWM, 4 channels) |
| TCS3200-style color sensor | Object color detection (TIM3 input capture + S0–S3 filter select) |
| RGB status LED | Visual indication of detected color |
| Electromagnet | Object pickup/release actuator |
| IR/photo "Box_detect" sensor | Detects object presence at pickup point (EXTI) |
| USART2 (115200 baud) | Debug/serial output |

## Peripheral Configuration

- **ADC1 + DMA1**: Scans 5 channels (ADC_CHANNEL_8–12) continuously in DMA mode into `adcBuffer[5]`; a conversion-complete callback sets an `adc_ready` flag consumed by the main loop.
- **TIM2**: 4-channel PWM (period 255) driving the two motors — each motor uses a forward/reverse channel pair.
- **TIM3**: Input capture on channel 1, used to measure the color sensor's output frequency for each filter (R/G/B).
- **USART2**: `printf`-redirected serial output for debug (color readings, etc.).
- **GPIO**: `Electromagnet`, `LD2` (onboard LED), `S0–S3` (color sensor frequency scaling/filter select), `RED/GREEN/BLUE` (status LED), `Box_detect` (EXTI input).

## Software Architecture

```
Core/
├── Inc/
│   ├── Color.h / Color.c   # TCS3200-style color sensor driver
│   ├── main.h
│   └── stm32f1xx_*.h
└── Src/
    ├── main.c              # Line-following PID + pickup/sort state machine
    ├── stm32f1xx_it.c      # Interrupt handlers
    └── stm32f1xx_hal_msp.c
Drivers/                    # STM32 HAL + CMSIS
```

### Line-Following PID

Five IR sensors are mapped to positions `{-2, -1, 0, 1, 2}`. Each loop iteration:

1. Normalizes ADC readings to `0.0–1.0`.
2. Computes a weighted-average error (position of the line relative to center).
3. Runs a PID controller:

   ```
   Kp = 1.5
   Ki = 0.055
   Kd = 0.4
   ```

4. Converts the PID output into differential motor speeds around a base speed (`base_speed = 0.37`), clamped to `[-1.0, 1.0]`.

A `mode` flag (0 = outbound, 1 = return) is toggled when both outer sensors simultaneously see a defined "gate" pattern for two consecutive samples, which also swaps which computed speed drives which motor (reversing effective direction of travel along the line).

### Pickup & Color Detection

When `Box_detect` triggers (object present) and the robot isn't already carrying an object:

1. Robot nudges forward briefly, then stops.
2. Electromagnet energizes.
3. `Color_Detect()` reads the object's color (returns `1 = RED`, `2 = GREEN`, `3 = BLUE`, `0 = MIXED/unknown`) and lights the matching status LED.
4. Robot performs a short scripted turn to rejoin the line.

### Junction Counting & Sorting

While in return mode, the robot counts junctions by detecting when the middle three IR sensors go dark for two consecutive samples. Depending on the detected object color, a different number of junctions must be crossed before the robot executes a scripted turn sequence to reach the correct bin and release the object (de-energizing the electromagnet).

Once two objects have been delivered, the robot halts and flashes its RGB LED white on a loop (`White_LED_box_Detect`) to indicate task completion.

## Building & Flashing

1. Import the project into **STM32CubeIDE** (`File → Import → Existing Projects into Workspace`).
2. Build the project (`Project → Build`).
3. Flash to an STM32F103RB Nucleo board over ST-Link (`Run → Debug` or `Run → Run`).

The `.ioc` file (`Task5b_PID.ioc`) can be opened in STM32CubeMX/CubeIDE to inspect or regenerate peripheral configuration.

## Tuning

If the robot oscillates or drifts off the line, adjust the PID gains and base speed in `main.c`:

```c
float Kp = 1.5f;
float Ki = 0.055f;
float Kd = 0.4f;
float base_speed = 0.37f;
```

Color detection thresholds (`COLOR_G_GAIN`, `COLOR_B_GAIN`, `COLOR_DIFF`) are defined in `Color.h` and may need recalibration for different lighting conditions or sensor units.

## Known Issues / TODO

- `set_motor()` computes PWM as `int16_t` but casts to `uint8_t` when writing to the timer compare register — safe at the current `TIM2` period (255) but fragile if speed scaling changes.
- Sorting logic branches on `color == 2` / `color == 3`; double-check these against your bin layout, since `Color_Detect()` returns `3` for blue, not red — naming in comments/state names may not match the numeric color codes.
- No debounce beyond the 2-sample counters for gate/junction detection; very fast runs may need tuning.

## License

Add your preferred license here (e.g., MIT).
