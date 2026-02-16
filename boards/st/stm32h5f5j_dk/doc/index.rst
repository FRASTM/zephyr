.. zephyr:board:: stm32h5f5j_dk

Overview
********

The STM32H5F5J-DK Discovery kit is designed as a complete demonstration and
development platform for STMicroelectronics Arm |reg| Cortex |reg|-M33 core-based
STM32H5F5LJH7Q microcontroller with TrustZone |reg|. Here are some highlights of
the STM32H5F5J-DK Discovery board:


- STM32H5F5LJH7Q microcontroller featuring 4 Mbytes of Flash memory and 1.5 Mbytes of SRAM in TFBGA225 package
- 4.3" RGB 480X572pixels TFT colored LCD module and touch panel
- USB Type-C |reg| Full-Speed device with USB power-delivery sink only controller
- SAI Audio DAC stereo with one audio jacks for input/output,
- ST MEMS digital microphone with PDM interface
- Octo-SPI interface connected to 512Mbit Octo-SPI NORFlash memory device (MX25LM51245GXDI00 from MACRONIX)
- Octo-SPI interface connected to 64Mbit Octo-SPI SRAM memory.
- 10/100-Mbit Ethernet,
- microSD  |trade|
- A Wi‑Fi® add-on board
- Board connectors

  - STMod+ expansion connector with fan-out expansion board for Wi‑Fi |reg|, Grove and mikroBUS |trade| compatible connectors
  - Pmod |trade| expansion connector
  - Audio MEMS daughterboard expansion connector
  - ARDUINO |reg| Uno V3 expansion connector

- Flexible power-supply options

  - ST-LINK
  - USB VBUS
  - external sources

- On-board STLINK-V3E debugger/programmer with USB re-enumeration capability:

  - mass storage
  - Virtual COM port
  - debug port

- 4 user LEDs
- User and reset push-buttons

More information about the board can be found at the `STM32H5F5J-DK Discovery website`_.

Hardware
********

The STM32H5F5xx devices are an high-performance microcontrollers family (STM32H5
Series) based on the high-performance Arm |reg| Cortex |reg|-M33 32-bit RISC core.
They operate at a frequency of up to 250 MHz.

- Core: ARM |reg| 32-bit Cortex |reg| -M33 CPU with TrustZone |reg| and FPU.
- Performance benchmark:

  - 375 DMPIS/MHz (Dhrystone 2.1)

- Security

  - Arm |reg| TrustZone |reg| with ARMv8-M mainline security extension
  - Up to 8 configurable SAU regions
  - TrustZone |reg| aware and securable peripherals
  - Flexible lifecycle scheme with secure debug authentication
  - Preconfigured immutable root of trust (ST-iROT)
  - SFI (secure firmware installation)
  - Secure data storage with hardware unique key (HUK)
  - Secure firmware upgrade support with TF-M
  - 2x AES coprocessors including one with DPA resistance
  - Public key accelerator, DPA resistant
  - On-the-fly decryption of Octo-SPI external memories
  - HASH hardware accelerator
  - True random number generator, NIST SP800-90B compliant
  - 96-bit unique ID
  - Active tampers
  - True Random Number Generator (RNG) NIST SP800-90B compliant

- Clock management:

  - 48 MHz crystal oscillator (HSE)
  - 32768 Hz crystal oscillator for RTC (LSE)
  - Internal 64 MHz (HSI) trimmable by software
  - Internal low-power 32 kHz RC (LSI)( |plusminus| 5%)
  - Internal 4 MHz oscillator (CSI), trimmable by software
  - Internal 48 MHz (HSI48) with recovery system
  - 3 PLLs for system clock, USB, audio, ADC

- Power management

  - Embedded regulator (LDO) with three configurable range output to supply the digital circuitry
  - Embedded SMPS step-down converter

- RTC with HW calendar, alarms and calibration
- Up to 178 fast I/Os, most 5 V-tolerant, up to 10 I/Os with independent supply down to 1.08 V
- Up to 24 timers and 2 watchdogs

  - 12x 16-bit
  - 2x 32-bit timers with up to 4 IC/OC/PWM or pulse counter and quadrature (incremental) encoder input
  - 6x 16-bit low-power 16-bit timers (available in Stop mode)
  - 2x watchdogs
  - 2x SysTick timer

- Memories

  - Up to 4 MB Flash, 2 banks read-while-write
  - 2 Kbyte OTP (one-time programmable)
  - 1536 KB of contiguous SRAM including 384 Kbytes with flexible ECC
  - 4 Kbytes of backup SRAM available in the lowest power modes
  - Flexible external memory controller with up to 32-bit data bus: SRAM, PSRAM, FRAM, SDRAM/LPSDR SDRAM, NOR/NAND memories
  - 2x OCTOSPI memory interface with on-the-fly decryption and support for serial PSRAM/NAND/NOR, Hyper RAM/Flash frame formats
  - 2x SD/SDIO/MMC interfaces

- Rich analog peripherals (independent supply)

  - 3x 12-bit ADC with up to 5 MSPS in 12-bit
  - 2x 12-bit D/A converters
  - 1x Digital temperature sensor

- 37x communication interfaces

  - 1x USB Type-C®/USB Power Delivery r3.1
  - 1x USB OTG full-speed
  - 1x USB OTG high-speed with embedded PHY
  - 4x I2C FM+ interfaces (SMBus/PMBus)
  - 2x I3C interface
  - 12x U(S)ARTS (ISO7816 interface, LIN, IrDA, modem control)
  - 1x LP UART
  - 6x SPIs including 3 muxed with full-duplex I2S
  - 5x additional SPI from 5x USART when configured in Synchronous mode
  - 2x SAI
  - 3x FDCAN
  - 1x SDMMC interface
  - 2x 16 channel DMA controllers
  - 1x 8- to 14- bit camera interface
  - 1x HDMI-CEC
  - 1x Ethernel MAC interface with DMA controller
  - 1x 16-bit parallel slave synchronous-interface

- CORDIC for trigonometric functions acceleration
- FMAC (filter mathematical accelerator)
- CRC calculation unit
- Development support: serial wire debug (SWD), JTAG, Embedded Trace Macrocell |trade|


More information about STM32H5F5 can be found here:

- `STM32H5F5 on www.st.com`_
- `STM32H5F5 reference manual`_

Supported Features
==================

.. zephyr:board-supported-hw::

Connections and IOs
===================

STM32H5F5J-DK Discovery Board has 9 GPIO controllers. These controllers are responsible for pin muxing,
input/output, pull-up, etc.

For more details please refer to `STM32H5F5J-DK Discovery board User Manual`_.

Default Zephyr Peripheral Mapping:
----------------------------------

- USART_2 TX/RX : PA2/PA3 (VCP)
- UART_7 TX/RX : PE8/PE7  (Arduino UART7)
- USER_PB : PI8
- LD1 (red) : PH10
- LD2 (green) : PA5
- LD3 (blue) : PB9
- LD4 (orange) : PD15
- ADC1 channel 0 input : PA0

System Clock
------------

STM32H5F5J-DK System Clock could be driven by internal or external oscillator,
as well as main PLL clock. By default System clock is driven by PLL clock at
250MHz, driven by 48MHz external oscillator (HSE).

Serial Port
-----------

STM32H5F5J-DK Discovery board has 3 U(S)ARTs. The Zephyr console output is
assigned to USART2. Default settings are 115200 8N1.

TFT LCD screen and touch panel
------------------------------

The TFT LCD screen and touch panel are supported for the STM32H5F5J-DK Discovery board.
They can be tested using :zephyr:code-sample:`lvgl` sample:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/display/lvgl
   :board: stm32h5f5j_dk
   :goals: build

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

STM32H5F5J-DK Discovery board includes an ST-LINK/V3E embedded debug tool interface.

Applications for the ``stm32h5f5j_dk`` board configuration can be built and
flashed in the usual way (see :ref:`build_an_application` and
:ref:`application_run` for more details).

OpenOCD Support
===============

For now, OpenOCD support  for STM32H5 is not available on upstream OpenOCD.
You can check `OpenOCD official Github mirror`_.
In order to use it though, you should clone from the customized
`STMicroelectronics OpenOCD Github`_ and compile it following usual README guidelines.
Once it is done, you can set the OPENOCD and OPENOCD_DEFAULT_PATH variables in
:zephyr_file:`boards/st/stm32h5f5j_dk/board.cmake` to point the build
to the paths of the OpenOCD binary and its scripts,  before
including the common openocd.board.cmake file:

   .. code-block:: none

      set(OPENOCD "<path_to_openocd_repo>/src/openocd" CACHE FILEPATH "" FORCE)
      set(OPENOCD_DEFAULT_PATH <path_to_opneocd_repo>/tcl)
      include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)


Flashing
========


The board is configured to be flashed using west `STM32CubeProgrammer`_ runner,
so its :ref:`installation <stm32cubeprog-flash-host-tools>` is required.

Alternatively, OpenOCD or pyOCD can also be used to flash the board using
the ``--runner`` (or ``-r``) option:

.. code-block:: console

   $ west flash --runner openocd
   $ west flash --runner pyocd

For pyOCD, additional target information needs to be installed
by executing the following commands:

.. code-block:: console

   $ pyocd pack --update
   $ pyocd pack --install stm32h5

Application in SoC Flash
========================

Connect the STM32H5F5J-DK Discovery to your host computer using the USB port.
Then build and flash an application. Here is an example for the
:zephyr:code-sample:`hello_world` application.

Run a serial host program to connect with your Nucleo board:

.. code-block:: console

   $ minicom -D /dev/ttyACM0

Then build and flash the application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: stm32h5f5j_dk
   :goals: build flash

You should see the following message on the console:

.. code-block:: console

   Hello World! stm32h5f5j_dk

Debugging
---------

Waiting for OpenOCD support, debugging could be performed with pyOCD which
requires to enable "pack" support with the following pyOCD command:

.. code-block:: console

   $ pyocd pack --update
   $ pyocd pack --install stm32h5

Once installed, you can debug an application in the usual way. Here is an
example for the :zephyr:code-sample:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: stm32h5f5j_dk
   :maybe-skip-config:
   :goals: debug

Application in External Flash
=============================

Since an external NOR is available on the board, you may want to use it to store
a large user application, and run it from there. In that case, the MCUboot bootloader
is needed to chainload the application. A dedicated board variant, ``ext_flash_app``, was created
for this usecase.

:ref:`sysbuild` makes it possible to build and flash all necessary images needed to run a user application
from external Flash.

The following example shows how to build :zephyr:code-sample:`hello_world` with Sysbuild enabled:

.. zephyr-app-commands::
   :tool: west
   :zephyr-app: samples/hello_world
   :board: stm32h5f5j_dk/stm32h5f5xx/ext_flash_app
   :goals: build
   :west-args: --sysbuild

By default, Sysbuild creates MCUboot and user application images.

For more information, refer to the :ref:`sysbuild` documentation.

Flashing
--------

Both MCUboot and user application images can be flashed by running:

.. code-block:: console

   west flash

You should see the following message in the serial host program:

.. code-block:: console

   *** Booting MCUboot v2.2.0-192-g96576b341ee1 ***
   *** Using Zephyr OS build v4.3.0-rc2-37-g6cc7bdb58a92 ***
   I: Starting bootloader
   I: Image index: 0, Swap type: none
   I: Image index: 0, Swap type: none
   I: Primary image: magic=unset, swap_type=0x1, copy_done=0x3, image_ok=0x3
   I: Secondary image: magic=unset, swap_type=0x1, copy_done=0x3, image_ok=0x3
   I: Boot source: none
   I: Image index: 0, Swap type: none
   I: Image index: 0, Swap type: none
   I: Image index: 0, Swap type: none
   I: Image index: 0, Swap type: none
   I: Bootloader chainload address offset: 0x0
   I: Image version: v0.0.0
   I: Jumping to the first image slot
   *** Booting Zephyr OS build v4.3.0-rc2-37-g6cc7bdb58a92 ***
   Hello World! stm32h5f5j_dk/stm32h5f5xx/ext_flash_app

To only flash the user application in the subsequent builds, Use:

.. code-block:: console

   west flash --domain hello_world

With the default configuration, the board uses MCUboot's Swap-using-offset mode.
To get more information about the different MCUboot operating modes and how to
perform application upgrade, refer to `MCUboot design`_.
To learn more about how to secure the application images stored in external Flash,
refer to `MCUboot Encryption`_.

Debugging
---------

You can debug the application in external flash using ``west`` and ``GDB``.

After flashing MCUboot and the app, execute the following command:

.. code-block:: console

   west debugserver

Then, open another terminal (don't forget to activate Zephyr's environment) and execute:

.. code-block:: console

   west attach

By default, user application symbols are loaded. To debug MCUboot application,
launch:

.. code-block:: console

   west attach --domain mcuboot

.. _STM32H5F5J-DK Discovery website:
   https://www.st.com/en/evaluation-tools/stm32h5f5j-dk.html

.. _STM32H5F5J-DK Discovery board User Manual:
   https://www.st.com/en/evaluation-tools/stm32h5f5j-dk.html

.. _STM32H5F5 on www.st.com:
   https://www.st.com/en/microcontrollers-microprocessors/stm32h5f5lj.html

.. _STM32H5F5 reference manual:
   https://www.st.com/resource/en/reference_manual/DM01023297.pdf

.. _STM32CubeProgrammer:
   https://www.st.com/en/development-tools/stm32cubeprog.html

.. _OpenOCD official Github mirror:
   https://github.com/openocd-org/openocd/

.. _STMicroelectronics OpenOCD Github:
   https://github.com/STMicroelectronics/OpenOCD/tree/openocd-cubeide-r6

.. _MCUboot design:
   https://docs.mcuboot.com/design.html

.. _MCUboot Encryption:
   https://docs.mcuboot.com/encrypted_images.html
