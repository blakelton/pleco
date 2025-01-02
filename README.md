# Pleco - Hand Vacuum Control System

Pleco is a small handvac that I decided to make for fun!

This repository contains the firmware, configuration details, wiring schimatics, and 3D models for a hand vacuum system based on the **STM32L432KC** MCU. The device is designed to manage a 12V PWM-controlled cooling fan for suction, display operating parameters on an LCD, handle user input for speed control, and monitor battery voltage. Additionally, 3D-printable models for the vacuum enclosure and components are included.

![Pleco Left Side](https://github.com/blakelton/pleco/blob/develop/Images/LSide.jpg?raw=true)

## Overview

The hand vacuum system includes:

- **STM32L432KC Nucleo-32 board** as the main controller.  
- A **12V PWM-controlled cooling fan** for creating vacuum suction.  
- An **I2C LCD 1602 display** for showing fan speed, battery status, and other operational data.  
- A **10kΩ linear potentiometer** for adjusting fan speed.  
- A **3S 11.1V LiPo battery** as the primary power source, with voltage monitoring.  
- **LM2596 DC-DC buck converter** to step down from ~12V battery level to 5V for the MCU and LCD.  
- **Resistors, fuse**, and other components for safe and stable operation.  
- **3D models** available in the `Models` folder for 3D printing the vacuum enclosure and related parts.

## Key Features

1. **Microcontroller (MCU):**  
   - **STM32L432KC (ARM Cortex-M4)** on a Nucleo-32 board.  
   - Provides PWM outputs for fan speed control.  
   - Reads ADC inputs for potentiometer and battery voltage.  
   - Manages I2C communication with the LCD display.  
   - Uses FreeRTOS for task scheduling and concurrency.

2. **Fan Control:**  
   - **12V 40mm PWM-controlled cooling fan**:  
     - PWM signal from MCU adjusts fan speed based on potentiometer input.  
     - Tachometer input to MCU measures fan RPM.  
     - If no pulse is detected for more than 1 second, RPM is set to zero indicating the fan is stopped.

3. **Display:**  
   - **I2C LCD 1602 module**:  
     - Displays fan RPM, battery voltage, and other statuses.  
     - Updated periodically by FreeRTOS tasks.

4. **User Input:**  
   - **10kΩ linear potentiometer** provides an analog voltage representing desired fan speed.

5. **Battery and Power Management:**  
   - **3S 11.1V 1000mAh LiPo Battery** as the main power source.  
   - **Voltage divider (100kΩ & 27kΩ)** and ADC input for battery voltage monitoring.  
   - **2A inline fuse** for overcurrent protection.  
   - **LM2596 buck converter** steps battery voltage down to 5V for the MCU and LCD.

6. **Switches and Connectors:**  
   - **SPST rocker switch** for main power.  
   - **DEAN/JST connectors** for battery connections.  
   - **Assorted wires (18 AWG for power, 22 AWG for signals)** ensure reliable wiring.

7. **3D Models:**  
   - The `Models` folder contains 3D models (STL or other formats) that can be used to 3D print the vacuum enclosure and related mechanical parts. Ensure proper print settings and material selection as per instructions or personal preference.

8. **Wiring Schematics:**  
   - The `Wiring` folder contains a rough wiring schematic for how this system was put together. This folder also includes a pinout for the MCU board. Both of these files are in PDF format.

9. **Software and Firmware:**  
   - **STM32Cube IDE & STM32CubeMX** for HAL configuration and FreeRTOS setup.  
   - **FreeRTOS** for task-based concurrent operations:  
     - Dedicated tasks for fan control, ADC monitoring, LCD updates, and RPM computations.

## Additional Features

- **PID Control:**  
  A robust PID algorithm is implemented for fan speed control, ensuring smooth operation and minimizing overshoot or oscillations.

- **Automated PID Testing:**  
  The firmware includes an optional auto-tuning procedure that tests different Kp, Ki, and Kd values over a defined duration. It computes a performance "score" for each set and picks the best combination for stable fan control.

- **Power Throttling for Low Power:**  
  When the battery voltage drops below a defined threshold, the system automatically reduces the duty cycle to zero, effectively shutting down the fan to prevent deep discharge of the battery.

- **Signal Noise Offset for ADC:**  
  The code applies a configurable noise offset (especially at higher RPM) to combat fluctuations in measured battery voltage, leading to more stable readouts.

## Calculating RPM

If the fan provides, for example, 4 pulses per revolution (PPR):

\[
\text{RPM} = \frac{\text{TimerFrequency} \times 60}{\text{PeriodTicks} \times \text{PulsesPerRev}}
\]

With a 1 MHz timer frequency, you can easily convert measured periods into RPM.

## Disclaimer

**Use this project at your own risk.** The author(s) and contributor(s) assume no liability or responsibility for any damage to property or personal injury that may result from assembling, operating, or modifying this device. Always follow proper safety guidelines, handle batteries carefully, and ensure correct wiring and fuse placement to prevent hazards.

## License

This project is licensed under the [MIT License](LICENSE). You are free to use, modify, and distribute this code under the terms of the MIT License.
