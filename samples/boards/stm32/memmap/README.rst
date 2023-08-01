.. _memmap:

FLASH NOR Sample in MemoryMapped
################################

Overview
********

This testcase is testing the stm32 flash API on a external NOR serial flash
memory device.
The Nor quad/octo Flash is configured in MemoryMapped mode at the device init
since the **CONFIG_STM32_MEMMAP** is set
Then read/write access is done with memcopy.
If accepted by the NOR, The testcase is writing few bytes and read them back at offset 0.
Then read bytes at offset=SPI_FLASH_SECTOR_SIZE

Building and Running
********************

The application will build only for a target that has a :ref:`devicetree
<dt-guide>` entry with ``zephyr,flash-controller`` as a compatible.

.. zephyr-app-commands::
   :zephyr-app: tests/drivers/flash/stm32/memmap
   :board: b_u585i_iot02a
   :goals: build flash
   :compact:

TestCase Output
===============

.. code-block:: console

   *** Booting Zephyr OS build zephyr-v2.3.0-2142-gca01d2e1d748  ***

   ospi-nor-flash@90000000 SPI flash testing
   ==========================

   Test 1: Flash erase
   Flash erase succeeded!

   Test 2: Flash write
   Attempting to write 32 bytes
   Data read matches data written. Good!

   Perform test on multiple consecutive sectors

   Data read :
   00000000 read 55
   00000001 read aa
   00000002 read 66
   00000003 read 99
   Data read :
   00001000 read ff
   00001001 read ff
   00001002 read ff
   00001003 read ff
