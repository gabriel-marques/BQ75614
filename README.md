# BQ75614 library

This readme explains some of the functionalities of the `BQ75614-Q1 14S or 16S Standalone Precision Automotive Battery Monitor, Balancer
and Integrated Current Sense` library from Texas Instrument and how to use it. This Battery Management System (BMS) has the following characteristics:

- Qualified for automotive applications
- AEC-Q100 Qualified with the following results:
    - Device temperature grade 1:  -40°C to +125°C
ambient operating temperature range
    - Device HBM ESD classification level 2
    - Device CDM ESD classification level C4B
- Functional Safety-Compliant
    - Developed for functional safety applications
    - Documentation to aid ISO 26262 system design
    - Systematic capability up to ASIL D
    - Hardware capability up to ASIL D
- +/- 1.5mV ADC accuracy
- Pin-package and software compatible device
family:
    - Stackable monitor 16S (BQ79616-Q1,
BQ79616H-Q1, BQ79656-Q1), 14S (BQ79614-
Q1, BQ79654-Q1), and 12S (BQ79612-Q1,
BQ79652-Q1)
    - Standalone monitor 48 V system (BQ75614-
Q1)
- Supports current-sense measurement
- Direct support on fuse and relay open and close
diagnostics
- Built-in redundancy path for voltage and
temperature and current diagnostics
- Highly accurate cell voltage measurements within
128 µs for all cell channels
- Integrated post-ADC configurable digital low-pass
filters
- Built-in host-controlled hardware reset to emulate
POR-like device reset
- Supports internal cell balancing
    - Balancing current at 240 mA
    - Built-in balancing thermal management with
automatic pause and resume control
- 5 V LDO output to power external digital isolator
- UART host interface
- Built-in SPI master

Datasheet (SLUSDT5B) can be found here [https://www.ti.com/lit/ds/slusdt5b/slusdt5b.pdf](https://www.ti.com/lit/ds/slusdt5b/slusdt5b.pdf).

It can be adapted to be used with the BQ79616 modules. If someone want to add the missing functions. The broadcast write and read are already implemented.

# Implementation methodology

This library is implemented in C and is platform independent. It means that we can easily add it to any project for any platform that has a UART port and uses C language by implementing only the interfacing functions. Examples: STM32, ESP32, NXP, etc.

Implementation is done in a manner to bring high level functions to user in order to make it easy to use and abstract him of all hardware specificities. Also, implementation is robust with error handling and data validation.

The best way to understand the program is to read function headers. The file `bq75614.h` contains all `enumeration`,`structure` and `function prototypes` with understandable names and functions headers are in the file `bq75614.c`.

# How to use

To use the library, just add the files:
- `bq75614.c`
- `bq75614.h`
- `bq75614_defines.h`
- `bq75614_registers.h`

to your project and include the file `bq75614.h` in your main file. You must then implement the following functions:

- `BQ75614_StatusType BQ75614_ReadUART(uint8_t *data, uint8_t size);`
- `BQ75614_StatusType BQ75614_WriteUART(uint8_t *data, uint8_t size);`
- `BQ75614_StatusType BQ75614_PingSetLowTime(BQ75614_HandleTypeDef *bq75614, int time);`
- `BQ75614_StatusType BQ75614_Delay(uint32_t ms);`

Those functions are the interface between the library and the hardware. Information about their implementation can be found in the file `bq75614.h`.

Also, you must set the shunt resistor value in the file `bq75614_defines.h` in the `BQ75614_SHUNT_RESISTOR_USER` define. This is also dependent of your hardware and is used to compute the current.

Finally, you must declare a variable of type `BQ75614_HandleTypeDef` and init it with the function `BQ75614_StatusType BQ75614_Init(BQ75614_HandleTypeDef *bq75614, BQ75614_ConfigStruct *bq75614_config)`. For that you must fill the `BQ75614_ConfigStruct` with the configuration of your BMS. A default configuration is provided with the macro `BQ75614_DEFAULT_CONFIG`. A lot of initialization will be done here, the main ADc will run, cell balancing, OTUT and OVUV will start, etc. Read carefully the content of `BQ75614_ConfigStruct` definition to understand what happens. After init, you should be good to go and use the BQ75614 functions.


# Actual state

A lot of implementation is done, but most aren't fully tested because the lack of time and now I don't have the hardware to test it anymore.

Implementation that needs to be done:
- All Built-In Self Test (BIST) functions -> that allow to test that the chip is working as expected
- Get fuse state
- hardware reset ping
- certainly more

Implemented but not tested:
- Get current
- low pass filters
- OverTemperature and UnderTemperature protections
- GPIO configuration
- probably more