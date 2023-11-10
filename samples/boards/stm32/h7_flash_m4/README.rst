.. zephyr:code-sample:: soc-flash-stm32h7
   :name: STM32H7 SoC flash
   :relevant-api: flash_interface flash_area_api

   Use the flash API to interact with the SoC flash.

Overview
********

This sample demonstrates using the :ref:`Flash API <flash_api>` on an SoC internal flash.
The sample uses :ref:`Flash map API <flash_map_api>` to obtain device for flash, using
DTS node label, and then directly uses :ref:`Flash API <flash_api>` to perform
flash operations.

Within the sample, user may observe how read/write/erase operations
are performed on a device, and how to first check whether device is
ready for operation.

Building and Running
********************

The application will build for any SoC with internal flash memory
access enabled, as it is default for SoC devices, and fixed-partition
defined over that internal flash labeled `scratch_partition`

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/soc_flash_stm32h7
   :board: stm32h747i-disco
   :goals: build flash
   :compact:

Sample Output
=============

.. code-block:: console
   *** Booting Zephyr OS build zephyr-v3.5.0-1187-g80dfcba8733d ***
   STM32H7 Flash Testing
   =====================
   Available partitions on SoC Flash
   image1_partition        @0, size 1048576
   image2_partition        @100000, size 786432
   scratch_partition       @1c0000, size 262144
Test 1: Flash erase page at 0x1c0000, size 262144
   Flash erase succeeded!

Test 2: Flash write
   Wrote 1024 blocks of 32 bytes
   Wrote 2048 blocks of 32 bytes
   Wrote 3072 blocks of 32 bytes
   Wrote 4096 blocks of 32 bytes
   Wrote 5120 blocks of 32 bytes
   Wrote 6144 blocks of 32 bytes
   Wrote 7168 blocks of 32 bytes
   Wrote 8192 blocks of 32 bytes
   read and verified upto 0x1c8000
   read and verified upto 0x1d0000
   read and verified upto 0x1d8000
   read and verified upto 0x1e0000
   read and verified upto 0x1e8000
   read and verified upto 0x1f0000
   read and verified upto 0x1f8000
   8192 blocks read and verified correctly

Test 3: Page layout API
   Offset  0x001e0004:
     belongs to the page 15 of start offset 0x001e0000
     and the size of 0x00020000 B.
   Page of number 15 has start offset 0x001e0000
     and size of 0x00020000 B.
     Page index resolved properly
   SoC flash consists of 16 pages.

Test 4: Write block size API
   write-block-size = 32
