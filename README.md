# Mazda Engine Coolant Temp 
***
This project is designed to display engine coolant temperature (ECT) on Mazda SkyActiv generation vehicles by controlling the display update based on the ignition signal.

The device is connected to the OBD2 diagnostic connector, since Mazda Skyactiv generation does not use gateway module and also has no diagnostic bus (except new models CX-30, CX-50, etc.), so the device can be connected anywhere to HS-CAN 500kbit/s bus.

**Development Environment:**
- CooCox CoIDE Version: 1.7.7

## Hardware Components
----------------------
| Component                                  | Name                              |
|--------------------------------------------|-----------------------------------|
| Microcontroller                            | [STM32F103C8T6][stm32-url]        |
| Display                                    | [SSD1306 I2C 128x32][oled-url]    |
| CAN Transceiver                            | [MCP2551-I/SN][can-url]           |
| Step-Down Voltage Regulator*               | [D24V50F5][pololu-V50-url]        |
| Resistor                                   | R1: 10 kΩ                         |

**Note:**
The step-Down Voltage Regulator [D24V50F5][pololu-V50-url] regulator is a relatively expensive option but offers comprehensive protection features. 
For a more cost-effective solution, consider the [S13V10F5][pololu-V10-url]. 
The [S13V30F5][pololu-V30-url] is an advanced version with reverse voltage protection up to 20V.

## Schematics
-------------
![Mazda-SkyActiv-EngineCoolantTemp](https://github.com/banyaszg/CarLogger-hw/assets/78595419/79c3e98f-28dc-44c9-aeb1-29fd081ac407)

## Pinout
---------
| STM32F103C8T6 | SSD1306       | MCP2551 | Resistor | Voltage Regulator | OBD2 |
|---------------|---------------|---------|----------|-------------------|------|
| 5V            |               | VDD     |          | VOUT              |      |
| 3.3V          | VCC           |         |          |                   |      |
| GND           | GND           | VSS     | R1       | GND               | 4-5  |
| B6            | SLC           |         |          |                   |      |
| B7            | SDA           |         |          |                   |      |
| B11           |               | RXD     |          |                   |      |
| B12           |               | TXD     |          |                   |      |
|               |               | RS      | R1       |                   |      |
|               |               | CANH    |          |                   | 6    |
|               |               | CANL    |          |                   | 14   |
|               |               |         |          | VIN               | 16   |

## Configuration
----------------
### CAN Bus
To set the CAN bus speed it is necessary to consider the clock frequency of the STM32 controller and set the appropriate parameters. 
The following settings are used in the code in `can.c`:

```c
CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
```

In `can.h`, the bus speed is configured for 500 Kb. The values are calculated for a 72 MHz clock.
```c
#define CAN1_SPEED_PRESCALE 9
```
It is important to keep in mind that if the frequency is changed, the time quanta must be recalculated:

CAN_BS1_3tq means that the phase of BS1 (first part of the bit) will be 3 time quanta.

CAN_BS2_4tq means that the phase of BS2 (second part of the bit) will be 4 time quanta.

These values are used to set the bit segments in the CAN frame and affect the baud rate of the CAN bus.
The values CAN_BS1 and CAN_BS2 can be selected depending on the required baud rate to ensure correct synchronization and minimize possible errors on the CAN bus.

## SSD1306 Display Driver

The SSD1306 display driver files used in this project were originally authored by [Tilen Majerle](tilen@majerle.eu). 
[Alexander Lutsai](s.lyra@ya.ru) has made modifications specifically for STM32f10x.

To modify the display resolution, navigate to the initialization section in the `ssd1306.c` file. 
#### License Information

The SSD1306 display driver files are distributed under the terms of the GNU General Public License as published by the Free Software Foundation. The full text of the license can be found [here](http://www.gnu.org/licenses/).

## Visual
---------
![Mazda_ECT](https://github.com/joemccann/dillinger/assets/78595419/b789ce9d-c7ea-414d-b633-f38f98338f01)


   [stm32-url]: <https://www.st.com/en/microcontrollers-microprocessors/stm32f103c8.html>
   [oled-url]: <https://www.solomon-systech.com/product/ssd1306/>
   [can-url]: <https://ww1.microchip.com/downloads/en/devicedoc/20001667g.pdf>
   [pololu-V50-url]: <https://www.pololu.com/product/2851>
   [pololu-v10-url]: <https://www.pololu.com/product/4083>
   [pololu-v30-url]: <https://www.pololu.com/product/4082>
