.. zephyr:code-sample:: fs
   :name: File system manipulation
   :relevant-api: file_system_api disk_access_interface

   Use file system API with various filesystems and storage devices.

Overview
********

This sample app demonstrates use of the file system API and uses the FAT, Ext2 or LittleFS file
system driver with SDHC card, SoC flash or external flash chip.

To access device the sample uses :ref:`disk_access_api`.

Requirements for SD card support
********************************

This project requires SD card support and microSD card formatted with proper file system
(FAT, Ext2 or LittleFS) See the :ref:`disk_access_api` documentation for Zephyr implementation details.
Boards that by default use SD card for storage: ``arduino_mkrzero``, ``esp_wrover_kit``,
``mimxrt1050_evk``, ``nrf52840_blip`` and  ``olimexino_stm32``. The sample should be able
to run with any other board that has "zephyr,sdmmc-disk" DT node enabled.

Requirements for setting up FAT FS on SoC flash
***********************************************

For the FAT FS to work with internal flash, the device needs to support erase
pages of size <= 4096 bytes and have at least 64kiB of flash available for
FAT FS partition alone.
Currently the following boards are supported:
``nrf52840dk/nrf52840``

Requirements for setting up FAT FS on external flash
****************************************************

This type of configuration requires external flash device to be available
on DK board. Currently following boards support the configuration:
``nrf52840dk/nrf52840`` by ``nrf52840dk_nrf52840_qspi`` configuration.

Building and Running FAT samples
********************************

Boards with default configurations, for example ``arduino_mkrzero`` or
``nrf52840dk/nrf52840`` using internal flash can be built using command:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/fs/fs_sample
   :board: nrf52840_blip
   :goals: build
   :compact:

Where used example board ``nrf52840_blip`` should be replaced with desired board.

In case when some more specific configuration is to be used for a given board,
for example ``nrf52840dk/nrf52840`` with MX25 device over QSPI, configuration
and DTS overlays need to be also selected. The command would look like this:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/fs/fs_sample
   :board: nrf52840dk/nrf52840
   :gen-args: -DEXTRA_CONF_FILE=nrf52840dk_nrf52840_qspi.conf -DDTC_OVERLAY_FILE=nrf52840dk_nrf52840_qspi.overlay
   :goals: build
   :compact:

In case when board with SD card is used FAT microSD card should be present in the
microSD slot. If there are any files or directories present in the card, the
sample lists them out on the debug serial output.

.. warning::
   In case when mount fails the device may get re-formatted to FAT FS.
   To disable this behaviour disable :kconfig:option:`CONFIG_FS_FATFS_MOUNT_MKFS` .

Building and Running EXT2 samples
*********************************

Ext2 sample can be built for ``hifive_unmatched/fu740/s7`` or ``bl5340_dvk/nrf5340/cpuapp``. Because
FAT is default file system for this sample, additional flags must be passed to build
the sample.

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/fs/fs_sample
   :board: hifive_unmatched/fu740/s7 hifive_unmatched/fu740/u74
   :gen-args: -DCONF_FILE=prj_ext.conf
   :goals: build
   :compact:

A microSD card must be present in a microSD card slot of the board, for the sample to execute.
After starting the sample a contents of a root directory should be printed on the console.

Building and Running LittleFS samples
*************************************

LittleFS sample can be built for boards with SD card support. Because
FAT is the default file system for this sample, additional flags must be passed to build
the sample with LittleFS support.

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/fs/fs_sample
   :board: nrf52840dk/nrf52840
   :gen-args: -DCONF_FILE=prj_littlefs.conf
   :goals: build
   :compact:

A microSD card must be present in a microSD card slot of the board, for the sample to execute.
The LittleFS configuration includes automatic drive formatting, which will erase any existing
data on the SD card. To disable this behavior, set ``CONFIG_FS_SAMPLE_FORMAT_DRIVE=n`` in your
configuration.

Drive Formatting
****************

This sample supports automatic drive formatting for all supported filesystems (FAT, EXT2, and LittleFS).
When enabled via ``CONFIG_FS_SAMPLE_FORMAT_DRIVE=y``, the sample will format the drive before
attempting to mount it. This ensures a clean filesystem but will erase all existing data.

.. warning::
   Drive formatting will erase all existing data on the storage device. Use with caution.

Building and Running LittleFS samples
*************************************

LittleFS sample can be built for boards that support SD card access. Because
FAT is default file system for this sample, additional flags must be passed to build
the sample with LittleFS support.

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/fs/fs_sample
   :board: <board>
   :gen-args: -DCONF_FILE=prj_littlefs.conf
   :goals: build
   :compact:

A microSD card must be present in a microSD card slot of the board, for the sample to execute.
The sample will use LittleFS with block device support to access the SD card.
After starting the sample, the contents of the root directory should be printed on the console.

.. note::
   LittleFS will create its file system structure on the SD card. If the card already
   contains a different file system, it may need to be reformatted or re-partitioned.
