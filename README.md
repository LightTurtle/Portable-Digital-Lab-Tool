# STM32G031K8 Multi-Function Test Instrument

**Digital Design Lab — Final Project (Fall 2025)**
**Authors:** Andrew Truong & Michael Mott

A bare-metal, register-level embedded system that combines a **power supply**, **ohmmeter**, and **function generator** into a single device, with mode switching via a 4×4 keypad and output displayed on dual 7-segment displays.

---

## Features

| Mode | Points | Description |
|------|--------|-------------|
| **Power Supply** | 1 pt | Adjustable 0–3.3 V DC output via PWM + RC filter |
| **Ohmmeter** | 1 pt | Measures resistance using ADC voltage divider (Ω and kΩ ranges) |
| **Function Generator** | 1 pt | Square wave output, 10 Hz – 50 kHz, user-selectable frequency |
| **4×4 Keypad Input** | 0.5 pt | Mode switching and parameter entry |
| **Dual 7-Segment Display** | 0.5 pt | 4-digit output via two MCP23017 I/O expanders over I2C |

## Hardware

### Microcontroller
- **STM32G031K8** (ARM Cortex-M0+, 16 MHz HSI clock)
- All peripherals configured at the register level — no HAL or standard peripheral libraries

### Pin Map

| Pin | Function |
|-----|----------|
| PA0 | ADC input (ohmmeter voltage divider) |
| PA6, PA7 | Keypad column outputs (Col 1, Col 2) |
| PA8 | TIM1_CH1 — PWM output (power supply) |
| PA9 | TIM1_CH2 — PWM output (function generator) |
| PA11 | I2C2_SCL (MCP23017 displays) |
| PA12 | I2C2_SDA (MCP23017 displays) |
| PB0, PB1 | Keypad column outputs (Col 3, Col 4) |
| PB2–PB5 | Keypad row inputs (with pull-ups) |
| PB6 | LED1 — Kilo indicator |
| PB7 | LED2 — Base indicator |

### External Components
- **2× MCP23017** I2C I/O expanders (addresses `0x20`, `0x21`)
- **2× HDSP-521A** dual 7-segment displays (common anode)
- **4×4 matrix keypad**
- **RC low-pass filter** on PA8 (10 kΩ + 10 µF recommended) for DC power supply output
- **10 kΩ reference resistor** for ohmmeter voltage divider
- **100 Ω series resistor** on PA9 function generator output

### Circuit Diagrams

**Power Supply RC Filter:**
```
PA8 → [R 10kΩ] → ┬ → DC OUTPUT
                  [C 10µF]
                  GND
```

**Ohmmeter Voltage Divider:**
```
3.3V → [R_ref 10kΩ] → ┬ → [R_unknown] → GND
                       PA0
```

---

## Operating Modes

### Power Supply Mode (Key A)
- **Display:** `VX.XX` (e.g., `V1.65` for 1.65 V)
- **Controls:**
  - `1` — Increment voltage by 0.05 V (max 3.3 V)
  - `2` — Decrement voltage by 0.05 V (min 0 V)
  - `4` — Quick preset: 0 V
  - `5` — Quick preset: 1.65 V
  - `6` — Quick preset: 3.3 V
- **LEDs:** LED2 (Base) on

### Ohmmeter Mode (Key B)
- **Display:** `OXXX` for ohms, `OX.XX` for kilohms, `OPEn` for open circuit
- Continuously measures resistance — no keypad controls needed
- **LEDs:** LED2 on for Ω range, LED1 on for kΩ range

### Function Generator Mode (Key C)
- **Display:** Frequency with 3 significant figures (e.g., `1.00` for 1 kHz)
- **Controls:**
  - `0–9` — Enter frequency digits (up to 5 digits)
  - `*` — Clear input
  - `#` — Confirm and set frequency
- **Valid range:** 10 Hz – 50,000 Hz
- **LEDs:** LED1 (Kilo) on for ≥1 kHz, LED2 (Base) on for <1 kHz

---

## Project Structure

```
Final Project/
├── FinalProject.c                  # Final combined source (all modes integrated)
├── combined_modes.c                # Earlier combined integration (development version)
├── test0_display_debug.c           # Test: I2C + MCP23017 display verification
├── test_keypad.c                   # Test: 4×4 keypad scanning with LED feedback
├── test_display_keypad_shift.c     # Test: Keypad input with calculator-style digit shifting
├── test2_power_supply_mode.c       # Test: Power supply mode standalone
├── test3_ohmmeter_mode.c           # Test: Ohmmeter mode standalone
├── test4_function_gen.c            # Test: Function generator mode standalone
├── test5_frequency_meter.c         # Test: Frequency meter (TIM3 input capture on PA1)
├── FinalProject.txt                # Project planning and design notes
└── README.md                       # This file
```

### Development Approach

Each feature was developed and tested independently before integration:

1. **test0** — Verified I2C communication and 7-segment display wiring with both MCP23017 chips
2. **test_keypad** — Validated 4×4 keypad scanning and debouncing
3. **test_display_keypad_shift** — Combined display + keypad with digit entry
4. **test2** — Power supply mode with PWM, display, and keypad control
5. **test3** — Ohmmeter mode with ADC measurement and auto-ranging display
6. **test4** — Function generator mode with frequency entry and timer configuration
7. **test5** — Frequency meter using TIM3 input capture
8. **combined_modes.c** — First integration of all modes with keypad switching
9. **FinalProject.c** — Final polished version

---

## Technical Details

### Peripherals Used
| Peripheral | Purpose |
|------------|---------|
| TIM1 CH1 | PWM for power supply (variable duty cycle → RC filter → DC) |
| TIM1 CH2 | PWM for function generator (variable frequency, 50% duty) |
| ADC (Channel 0) | Reads voltage divider for ohmmeter |
| I2C2 | Communicates with MCP23017 I/O expanders |
| GPIO | Keypad scanning, LED indicators |

### I2C Error Handling
The I2C write function includes timeout-based error handling with NACK detection to prevent bus lockup, plus a bus recovery routine that bit-bangs the SCL/SDA lines on startup.

### PWM Frequency Ranges (Function Generator)
| Frequency Range | Prescaler | Timer Clock |
|-----------------|-----------|-------------|
| 10–100 Hz | 1599 | 10 kHz |
| 100–1,000 Hz | 159 | 100 kHz |
| 1–10 kHz | 15 | 1 MHz |
| 10–50 kHz | 0 | 16 MHz |

---

## Building & Flashing

This project is written in bare-metal C with no external dependencies. Build with your ARM toolchain and flash via ST-Link:

```bash
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -nostdlib -T linker.ld -o FinalProject.elf FinalProject.c
arm-none-eabi-objcopy -O binary FinalProject.elf FinalProject.bin
st-flash write FinalProject.bin 0x08000000
```

> Adjust the linker script and build commands to match your specific toolchain setup.
