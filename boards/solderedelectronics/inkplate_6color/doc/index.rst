.. zephyr:board:: inkplate_6color

Overview
********

The Inkplate 6Color is an ESP32-based development board featuring a 6.2-inch 
7-color e-paper display designed and manufactured by Soldered Electronics. This 
board combines the powerful ESP32 dual-core microcontroller with a large, 
high-resolution e-paper display, making it ideal for applications requiring 
low-power, always-on visual displays such as dashboards, information panels, 
and IoT displays.

The board features both WiFi and Bluetooth connectivity through the ESP32, 
along with extensive I/O capabilities, real-time clock functionality, and 
microSD card storage support.

Hardware
********

The Inkplate 6Color is built around the ESP32-WROVER-E module and includes 
the following key components and features:

**ESP32-WROVER-E Specifications:**

- Dual-core Xtensa LX6 microprocessor running at up to 240 MHz
- 4 MB SPI flash memory  
- 8 MB external PSRAM
- 802.11 b/g/n Wi-Fi (2.4 GHz)
- Bluetooth v4.2 BR/EDR and BLE
- Ultra-low power consumption with multiple power modes

**Display:**

- 6.2-inch 7-color e-paper display
- Resolution: 1200 × 825 pixels
- Colors: Black, white, red, yellow, green, blue, orange
- Ultra-low power consumption (only consumes power during refresh)

**Key Components:**

- **PCAL6416A**: 16-bit I2C GPIO expander for additional I/O pins
- **PCF85063A**: Real-time clock (RTC) with battery backup capability
- **MicroSD Card Slot**: For external storage (SPI interface)
- **Wake-up Button**: User button for device wake-up
- **Battery Voltage Monitor**: ADC-based battery level monitoring
- **easyC Connector**: Soldered Electronics' standardized I2C connector

**Power Management:**

- USB-C connector for power and programming
- Battery connector for portable operation
- Power management with multiple sleep modes
- Battery voltage monitoring through ADC

**Digital Interfaces:**

- Multiple GPIO pins available through expansion headers
- 2× I2C buses (one for internal components, one for expansion)
- 2× SPI buses (one for SD card, one for e-paper display)
- UART for console and debugging
- ADC channels for analog sensing

**Connectivity:**

- Wi-Fi 802.11 b/g/n (2.4 GHz) with integrated antenna
- Bluetooth v4.2 BR/EDR and BLE
- easyC connector for I2C device expansion

Supported Features
******************

.. zephyr:board-supported-hw::

The following interfaces are supported:

- **GPIO**: General-purpose I/O pins with interrupt capability
- **I2C**: Two I2C buses for sensor and peripheral communication  
- **SPI**: SPI interface for SD card and display communication
- **UART**: Serial communication for console and debugging
- **ADC**: Analog-to-digital converter for battery monitoring
- **RTC**: Real-time clock with alarm functionality
- **Watchdog**: Hardware watchdog timer
- **Wi-Fi**: 802.11 b/g/n wireless networking
- **Bluetooth**: Bluetooth Classic and BLE connectivity
- **NVS**: Non-volatile storage in flash memory

Board Variants
==============

The Inkplate 6Color has two board variants corresponding to the ESP32's dual cores:

- ``inkplate_6color/esp32/procpu``: The main processor core (PRO_CPU)
- ``inkplate_6color/esp32/appcpu``: The application processor core (APP_CPU)

For most applications, use the PRO_CPU variant which has access to all peripherals 
and connectivity features.

Connections and IOs  
===================

The board provides the following key connections:

**Power:**
- USB-C connector for power and programming
- JST connector for battery power
- Power switch

**easyC Connector (I2C):**
- Pin 1: GND
- Pin 2: VCC (3.3V)  
- Pin 3: SDA (GPIO21)
- Pin 4: SCL (GPIO22)

**Expansion Headers:**
GPIO pins are available through expansion headers for custom interfacing.

**Internal I2C Bus (Bus 0):**

+---------------+---------+----------------------------------+
| Device        | Address | Function                         |
+===============+=========+==================================+
| PCAL6416A     | 0x20    | 16-bit GPIO expander             |
+---------------+---------+----------------------------------+
| PCF85063A     | 0x51    | Real-time clock                  |
+---------------+---------+----------------------------------+

**GPIO Assignments:**

+--------+------------------+----------------------------------+
| GPIO   | Function         | Usage                            |
+========+==================+==================================+
| GPIO1  | UART0_TX         | Console UART transmit            |
+--------+------------------+----------------------------------+
| GPIO3  | UART0_RX         | Console UART receive             |
+--------+------------------+----------------------------------+
| GPIO12 | SPI2_MISO        | SD card SPI interface            |
+--------+------------------+----------------------------------+
| GPIO13 | SPI2_MOSI        | SD card SPI interface            |
+--------+------------------+----------------------------------+
| GPIO14 | SPI2_SCLK        | SD card SPI interface            |
+--------+------------------+----------------------------------+
| GPIO15 | SPI2_CS          | SD card chip select              |
+--------+------------------+----------------------------------+
| GPIO21 | I2C0_SDA         | External I2C data (easyC)        |
+--------+------------------+----------------------------------+
| GPIO22 | I2C0_SCL         | External I2C clock (easyC)       |
+--------+------------------+----------------------------------+
| GPIO26 | I2C1_SDA         | Internal I2C data                |
+--------+------------------+----------------------------------+
| GPIO27 | SPI3_MOSI        | E-paper display interface        |
+--------+------------------+----------------------------------+
| GPIO32 | I2C1_SCL         | Internal I2C clock               |
+--------+------------------+----------------------------------+
| GPIO36 | Wake Button      | User wake-up button (active low)|
+--------+------------------+----------------------------------+
| GPIO39 | RTC Interrupt    | PCF85063A interrupt output       |
+--------+------------------+----------------------------------+

System Requirements
*******************

Espressif HAL requires Wi-Fi and Bluetooth binary blobs to work properly. Run 
the command below to retrieve those files:

.. code-block:: console

   west blobs fetch hal_espressif

.. note::

   It is recommended to run the command above after :file:`west update`.

Programming and Debugging  
*************************

.. zephyr:board-supported-runners::

The board can be programmed using the built-in USB-to-serial converter via the 
USB-C connector.

Building Applications
=====================

Build applications for the Inkplate 6Color using the PRO_CPU variant (recommended 
for most applications):

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: inkplate_6color/esp32/procpu  
   :goals: build flash

For dual-core applications, you can build for the APP_CPU variant:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: inkplate_6color/esp32/appcpu
   :goals: build flash

Flashing
========

The board supports flashing via the built-in USB-to-serial converter. Connect 
the USB-C cable and flash as shown above.

If the board does not enter programming mode automatically, you may need to hold 
the boot button while pressing the reset button, then release both buttons.

Debugging
=========

The ESP32 on the Inkplate 6Color supports debugging via OpenOCD and JTAG. The 
JTAG pins are not brought out to a standard connector and require manual wiring 
to an external JTAG adapter.

For more information about ESP32 debugging, refer to the `ESP32 JTAG Debugging Guide`_.

References
**********

.. target-notes::

.. _ESP32 Datasheet: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
.. _ESP32 Technical Reference Manual: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf  
.. _ESP32 JTAG Debugging Guide: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/jtag-debugging/index.html
.. _Soldered Electronics: https://soldered.com/
.. _PCAL6416A Datasheet: https://www.nxp.com/docs/en/data-sheet/PCAL6416A.pdf
.. _PCF85063A Datasheet: https://www.nxp.com/docs/en/data-sheet/PCF85063A.pdf
