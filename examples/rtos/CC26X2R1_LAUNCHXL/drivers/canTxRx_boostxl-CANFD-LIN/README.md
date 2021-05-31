## Example Summary

The canTxRx_boostxl-CANFD-LIN example application transmits the CAN ID 0x123 every 100ms. And it toggles their data content (byte 0) when it receives the CAN ID 0x200. 

## Peripherals & Pin Assignments

When this project is built, the SysConfig tool will generate the TI-Driver
configurations into the __ti_drivers_config.c__ and __ti_drivers_config.h__
files. Information on pins and resources used is present in both generated
files. Additionally, the System Configuration file (\*.syscfg) present in the
project may be opened with SysConfig's graphical user interface to determine
pins and resources used.

* `CONFIG_GPIO_LED_0` - Indicator LED when CAN message 0x123 is transmitted
* `CONFIG_GPIO_LED_1` - Indicator LED when CAN message 0x200 is received
* `Board_GPIO_TCAN4550_CS` - TCAN4550 chip select pin
* `Board_TCAN4550_IRQ` - TCAN4550 interrupt pin
* `Board_GPIO_TLIN_EN` - TLIN enable/disable pin
* `Board_SPI0` - TCAN4550 SPI interface 

## BoosterPacks, Board Resources & Jumper Settings

This example requires a
[__SPI to CAN FD SBC + LIN transceiver BoosterPack™ plug-in module__][BOOSTXL-CANFD-LIN].

For board specific jumper settings, resources and BoosterPack modifications,
refer to the __Board.html__ file.

> If you're using an IDE such as Code Composer Studio (CCS) or IAR, please
refer to Board.html in your project directory for resources used and
board-specific jumper settings.

The Board.html can also be found in your SDK installation:

        <SDK_INSTALL_DIR>/source/ti/boards/<BOARD>

## Example Usage

* Example output requires a CAN sniffer with the following settings:

```

    Bit Rate:       500 kBit/s
    Sample Point:   81.25%
    Tq:             125ns
    Time Qunata:    16
```

* Run the example. `CONFIG_GPIO_LED_0` will toggle indicating that message 0x123
was sent.

```

     Chn     ID    Event Type   Dir    DLC   Data length   Data                          
     CAN 1   123   CAN Frame    Rx     8     8             01 02 03 04 05 06 07 08                                               
     CAN 1   220   CAN Frame    Tx     8     8             00 00 00 00 00 00 00 00
     CAN 1   123   CAN Frame    Rx     8     8             00 02 03 04 05 06 07 08
     CAN 1   123   CAN Frame    Rx     8     8             00 02 03 04 05 06 07 08                                                   
     CAN 1   123   CAN Frame    Rx     8     8             00 02 03 04 05 06 07 08
     CAN 1   220   CAN Frame    Tx     8     8             00 00 00 00 00 00 00 00

```
`CONFIG_GPIO_LED_1` will toggle indicating that message 0x200 was received.

## Application Design Details

This application uses one thread:

`CAN_taskFxn` - performs the following actions:

1. Opens and initializes SPI and GPIO.

2. Initializes the TCAN4550 over SPI and TCAN4550 IRQ (can_drv).

3. Transmits message 0x123 every 100ms.

4. Process message 0x220 "if received", and modifies byte 0 from message 0x123.
	* From 0x00 to 0x01
	* From 0x01 to 0x00 


TI-RTOS:

* When building in Code Composer Studio, the kernel configuration project will
be imported along with the example. The kernel configuration project is
referenced by the example, so it will be built first. The "release" kernel
configuration is the default project used. It has many debug features disabled.
These feature include assert checking, logging and runtime stack checks. For a
detailed difference between the "release" and "debug" kernel configurations and
how to switch between them, please refer to the SimpleLink MCU SDK User's
Guide. The "release" and "debug" kernel configuration projects can be found
under &lt;SDK_INSTALL_DIR&gt;/kernel/tirtos/builds/&lt;BOARD&gt;/(release|debug)/(ccs|gcc).


[BOOSTXL-CANFD-LIN]: https://www.ti.com/tool/BOOSTXL-CANFD-LIN
