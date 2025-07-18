.. _programming_thingy:

Updating the Thingy:91 firmware using the Programmer app
########################################################

.. contents::
   :local:
   :depth: 2

You can use the Programmer app from `nRF Connect for Desktop`_ for the following:

* :ref:`Update the Connectivity bridge application firmware in the nRF52840 SoC <updating_the conn_bridge_52840>`.
* :ref:`Update the modem firmware on the nRF9160 SiP <update_modem_fw_nrf9160>`.
* :ref:`Program the application firmware on the nRF9160 SiP <update_nrf9160_application>`.

These operations can be done through USB using MCUboot, or through an external debug probe.
When developing with your Thingy:91, it is recommended to use an external debug probe.

.. note::
   The external debug probe must support Arm Cortex-M33, such as the nRF9160 DK.
   You need a 10-pin 2x5 socket-socket 1.27 mm IDC (:term:`Serial Wire Debug (SWD)`) JTAG cable to connect to the external debug probe.

Download and extract the latest application and modem firmware from the `Thingy:91 Downloads`_ page.

The downloaded ZIP archive contains the following firmware:

Application firmware
  The :file:`img_app_bl` folder contains full firmware images for different applications.
  The guides for programming through an external debug probe in this section use the images in this folder.

Application firmware for Device Firmware Update (DFU)
  The images in the :file:`img_fota_dfu_bin` and :file:`img_fota_dfu_hex` folders contain firmware images for DFU.
  The guides for programming through USB in this section use the images in the :file:`img_fota_dfu_hex` folder.

Modem firmware
  The modem firmware is in a ZIP archive instead of a folder.
  The archive is named :file:`mfw_nrf9160_` followed by the firmware version number.
  Do not unzip this file.

The :file:`CONTENTS.txt` file in the extracted folder contains the location and names of the different firmware images.

The instructions in this section show you how to program the :ref:`connectivity_bridge` application and the :ref:`at_client_sample` sample, as well as the modem firmware.

The data is transmitted using either LTE-M or NB-IoT.
The :ref:`at_client_sample` sample first attempts to use LTE-M, then NB-IoT.
Check with your SIM card provider for the mode they support at your location.
For the iBasis SIM card provided with the Thingy:91, see `iBasis IoT network coverage`_.

.. tip::
   For a more compact nRF Cloud firmware application, you can build and install the :ref:`nrf_cloud_multi_service` sample.
   See the :ref:`building_pgming` section for more information.

.. note::
   To update the Thingy:91 through USB, the nRF9160 SiP and nRF52840 SoC bootloaders must be factory-compatible.
   The bootloaders might not be factory-compatible if the nRF9160 SiP or nRF52840 SoC has been updated with an external debug probe.
   To restore the bootloaders, program the nRF9160 SiP or nRF52840 SoC with factory-compatible Thingy:91 firmware files through an external debug probe.

.. note::
   You can also use these precompiled firmware image files for restoring the firmware to its initial image.

.. _updating_the conn_bridge_52840:

Updating the firmware in the nRF52840 SoC
*****************************************

.. tabs::

   .. group-tab:: Through USB

      To update the firmware, complete the following steps:

      1. Open `nRF Connect for Desktop`_ and launch the Programmer app.
      #. Scroll down in the menu on the left and make sure **Enable MCUboot** is selected.

         .. figure:: images/programmer_enable_mcuboot.png
            :alt: Programmer - Enable MCUboot

            Programmer - Enable MCUboot

      #. Switch off the Thingy:91.
      #. Press **SW4** while switching **SW1** to the **ON** position.

         .. figure:: images/thingy91_sw1_sw4.webp
            :alt: thingy91_sw1_sw4
            :width: 515px

            Thingy:91 - SW1 SW4 switch

      #. In the Programmer navigation bar, click :guilabel:`SELECT DEVICE`.
         A drop-down menu appears.

         .. figure:: images/programmer_select_device2.png
            :alt: Programmer - Select device

            Programmer - Select device

      #. In the menu, select the entry corresponding to your device (:guilabel:`MCUBOOT`).

         .. note::
            The device entry might not be the same in all cases and can vary depending on the application version and the operating system.

      #. In the menu on the left, click :guilabel:`Add file` in the **FILE** section, and select :guilabel:`Browse`.
         A file explorer window appears.

         .. figure:: images/programmer_add_file2.png
            :alt: Programmer - Add file

            Programmer - Add file

      #. Navigate to where you extracted the firmware.

      #. Open the folder :file:`img_fota_dfu_hex` that contains the HEX files for updating over USB.
         See the :file:`CONTENTS.txt` file for information on which file you need.

      #. Select the Connectivity bridge firmware file.

      #. Click :guilabel:`Open`.

      #. Scroll down in the menu on the left to the **DEVICE** section and click :guilabel:`Write`.

         .. figure:: images/programmer_hex_write1.png
            :alt: Programmer - Writing of HEX files

            Programmer - Writing of HEX files

         The **MCUboot DFU** window appears.

         .. figure:: images/thingy91_mcuboot_dfu.png
            :alt: Programmer - MCUboot DFU

            Programmer - MCUboot DFU

      #. In the **MCUboot DFU** window, click :guilabel:`Write`.
         When the update is complete, a "Completed successfully" message appears.
      #. Scroll up in the menu on the left to the **FILE** section and click :guilabel:`Clear files`.

   .. group-tab:: Through external debug probe

      To update the firmware using the nRF9160 DK as the external debug probe, complete the following steps:

      1. Open `nRF Connect for Desktop`_ and launch the Programmer app.

      .. _prepare_hw_ext_dp:

      2. Prepare the hardware:

         a. Connect the Thingy:91 to the debug out port on a 10-pin external debug probe using a JTAG cable.

            .. figure:: images/programmer_thingy91_connect_dk_swd_vddio.webp
               :alt: Thingy:91 - Connecting the external debug probe
               :width: 626px

               Thingy:91 - Connecting the external debug probe

            .. note::
               When using nRF9160 DK as the debug probe, make sure that VDD_IO (SW11) is set to 1.8 V on the nRF9160 DK.

         #. Make sure that the Thingy:91 and the external debug probe are powered on.

            .. note::
               Do not unplug or power off the devices during this process.

         #. Connect the external debug probe to the computer with a micro-USB cable.

            In the Programmer navigation bar, :guilabel:`No devices available` changes to :guilabel:`SELECT DEVICE`.

            .. figure:: images/programmer_select_device1.png
               :alt: Programmer - Select device

               Programmer - SELECT DEVICE
         #. Click :guilabel:`SELECT DEVICE` and select the appropriate debug probe entry from the drop-down list.

            Select nRF9160 DK from the list.

            .. figure:: images/programmer_com_ports.png
               :alt: Programmer - nRF9160 DK

               Programmer - nRF9160 DK

            The button text changes to the SEGGER ID of the selected device, and the **Device memory layout** section indicates that the device is connected.

      #. Set the SWD selection switch **SW2** to **nRF52** on the Thingy:91.
         See `SWD Select`_ for more information on the switch.

      #. In the menu on the left, click :guilabel:`Add file` in the **FILE** section, and select :guilabel:`Browse`.
         A file explorer window appears.

         .. figure:: images/programmer_add_file1.png
            :alt: Programmer - Add file

            Programmer - Add file

      #. Navigate to where you extracted the firmware.

      #. Open the folder :file:`img_app_bl` that contains the HEX files for flashing with a debugger.
         See the :file:`CONTENTS.txt` file for information on which file you need.

      #. Select the Connectivity bridge firmware file.
      #. Click :guilabel:`Open`.
      #. Scroll down in the menu on the left to the **DEVICE** section and click :guilabel:`Erase & write`.
         The update is completed when the animation in Programmer's **Device memory layout** window ends.

         .. figure:: images/programmer_ext_debug_hex_write.png
            :alt: Programming using an external debug probe

            Programming using an external debug probe

      #. Scroll up in the menu on the left to the **FILE** section and click :guilabel:`Clear files`.

.. _update_modem_fw_nrf9160:

Update the modem firmware on the nRF9160 SiP
********************************************

.. tabs::

   .. group-tab:: Through USB

     To update the modem firmware using USB, complete the following steps:

      1. Open `nRF Connect for Desktop`_ and launch the Programmer app if you do not have it open already.
      #. Make sure that **Enable MCUboot** is selected.
      #. Switch off the Thingy:91.
      #. Press **SW3** while switching **SW1** to the **ON** position.

         .. figure:: images/thingy91_sw1_sw3.webp
            :alt: Thingy:91 - SW1 SW3 switch
            :width: 483px

            Thingy:91 - SW1 SW3 switch

      #. In the menu, select Thingy:91.

      #. In the menu on the left, click :guilabel:`Add file` in the **FILE** section, and select :guilabel:`Browse`.
         A file explorer window appears.

         .. figure:: images/programmer_add_file.png
            :alt: Programmer - Add file

            Programmer - Add file

      #. Navigate to where you extracted the firmware.
      #. Find the modem firmware zip file with the name similar to :file:`mfw_nrf9160_*.zip` and the number of the latest version.

         .. note::
            Do not extract the modem firmware zip file.

      #. Select the zip file and click :guilabel:`Open`.
      #. In the Programmer app, scroll down in the menu on the left to the **DEVICE** section and click :guilabel:`Write`.

         .. figure:: images/programmer_usb_update_modem.png
            :alt: Programmer - Update modem

            Programmer - Update modem

         The **Modem DFU via MCUboot** window appears.

         .. figure:: images/thingy91_modemdfu_mcuboot.png
            :alt: Programmer - Modem DFU via MCUboot

            Programmer - Modem DFU via MCUboot

      #. In the **Modem DFU via MCUboot** window, click :guilabel:`Write`.
         When the update is complete, a **Completed successfully** message appears.

   .. group-tab:: Through external debug probe

      To update the modem firmware using an external debug probe, complete the following steps:

      1. Open `nRF Connect for Desktop`_ and launch the Programmer app and :ref:`prepare the hardware <prepare_hw_ext_dp>` if you have not done it already.
      #. Set the SWD selection switch **SW2** to **nRF91** on the Thingy:91.

      #. In the menu on the left, click :guilabel:`Add file` in the **FILE** section, and select :guilabel:`Browse`.
         A file explorer window appears.

         .. figure:: images/programmer_add_file1.png
            :alt: Programmer - Add file

            Programmer - Add file

      #. Navigate to where you extracted the firmware.
      #. Find the modem firmware zip file with the name similar to :file:`mfw_nrf9160_*.zip` and the number of the latest version and click :guilabel:`Open`.

         .. note::
            Do not extract the modem firmware zip file.

      #. Select the zip file and click :guilabel:`Open`.
      #. In the Programmer app, scroll down in the menu on the left to the **DEVICE** section and click :guilabel:`Write`.

         .. figure:: images/programmer_ext_debug_update_modem.png
            :alt: Programmer - Update modem

            Programmer - Update modem

         The **Modem DFU** window appears.

         .. figure:: images/programmer_modemdfu.png
            :alt: Programmer - Modem DFU

            Programmer - Modem DFU

      #. In the **Modem DFU** window, click :guilabel:`Write`.
         When the update is complete, a "Completed successfully" message appears.

         .. note::
            Before trying to update the modem again, click the :guilabel:`Erase all` button. In this case, the contents of the flash memory are deleted and the applications must be reprogrammed.

.. _update_nrf9160_application:

Program the nRF9160 SiP application
***********************************

.. tabs::

   .. group-tab:: Through USB

      To program the application firmware using USB, complete the following steps:

      1. Open `nRF Connect for Desktop`_ and launch the Programmer app if you have not done already.
      #. Make sure that **Enable MCUboot** is selected.
      #. Switch off the Thingy:91.
      #. Press **SW3** while switching **SW1** to the **ON** position.

         .. figure:: images/thingy91_sw1_sw3.webp
            :alt: Thingy:91 - SW1 SW3 switch
            :width: 483px

            Thingy:91 - SW1 SW3 switch

      #. In the Programmer navigation bar, click :guilabel:`SELECT DEVICE`.
         A drop-down menu appears.

         .. figure:: images/programmer_select_device.png
            :alt: Programmer - Select device

            Programmer - Select device

      #. In the menu, select Thingy:91.

      #. In the menu on the left, click :guilabel:`Add file` in the **FILE** section, and select :guilabel:`Browse`.
         A file explorer window appears.

         .. figure:: images/programmer_add_file.png
            :alt: Programmer - Add file

            Programmer - Add file

      #. Navigate to where you extracted the firmware.

      #. Open the folder :file:`img_fota_dfu_hex` that contains the HEX files for updating over USB.
         See the :file:`CONTENTS.txt` file for information on which file you need.

      #. Select the appropriate AT Client firmware file.

      #. Click :guilabel:`Open`.

      #. Scroll down in the menu on the left to the **DEVICE** section and click :guilabel:`Write`.

         .. figure:: images/programmer_hex_write.png
            :alt: Programmer - Writing of HEX files

            Programmer - Writing of HEX files

         The **MCUboot DFU** window appears.

         .. figure:: images/thingy91_mcuboot_dfu1.png
            :alt: Programmer - MCUboot DFU

            Programmer - MCUboot DFU

      #. In the **MCUboot DFU** window, click :guilabel:`Write`.
         When the update is complete, a **Completed successfully** message appears.
      #. Scroll up in the menu on the left to the **FILE** section and click :guilabel:`Clear files`.

   .. group-tab:: Through external debug probe

      To program the application firmware using an external debug probe, complete the following steps:

      1. Open `nRF Connect for Desktop`_ and launch the Programmer app and :ref:`prepare the hardware <prepare_hw_ext_dp>` if you have not done it already.
      #. Make sure the SWD selection switch **SW2** is set to **nRF91** on the Thingy:91.

      #. In the menu on the left, click :guilabel:`Add file` in the **FILE** section, and select :guilabel:`Browse`.
         A file explorer window appears.

         .. figure:: images/programmer_add_file1.png
            :alt: Programmer - Add file

            Programmer - Add file

      #. Navigate to where you extracted the firmware.

      #. Open the folder :file:`img_app_bl` that contains the HEX files for updating using a debugger.
         See the :file:`CONTENTS.txt` file for information on which file you need.

      #. Select the appropriate AT Client firmware file.

      #. Click :guilabel:`Open`.
      #. Scroll down in the menu on the left to the **DEVICE** section and click :guilabel:`Erase & write`.
         The update is completed when the animation in Programmer's **Device memory layout** window ends.

         .. figure:: images/programmer_ext_debug_hex_write.png
            :alt: Programming using an external debug probe

            Programming using an external debug probe

      #. Scroll up in the menu on the left to the **FILE** section and click :guilabel:`Clear files`.

.. _thingy91_partition_layout:

Partition layout
****************

When building firmware on the Thingy:91 board, a static partition layout matching the factory layout is used.
This ensures that programming firmware through USB works.
In this case, the MCUboot bootloader will not be updated.
So, to maintain compatibility, it is important that the image partitions do not get moved.
When programming the Thingy:91 through an external debug probe, all partitions, including MCUboot, are programmed.
This enables the possibility of using an updated bootloader or defining an application-specific partition layout.

Configure the partition layout using one of the following configuration options:

* :kconfig:option:`CONFIG_THINGY91_STATIC_PARTITIONS_FACTORY` - This option is the default Thingy:91 partition layout used in the factory firmware.
  This ensures firmware updates are compatible with Thingy:91 when programming firmware through USB.
* :kconfig:option:`CONFIG_THINGY91_STATIC_PARTITIONS_SECURE_BOOT` - This option is similar to the factory partition layout, but also has space for the immutable bootloader and two MCUboot slots.
  A debugger is needed to program Thingy:91 for the first time.
  This is an :ref:`experimental <software_maturity>` feature.
* :kconfig:option:`CONFIG_THINGY91_STATIC_PARTITIONS_LWM2M_CARRIER` - This option uses a partition layout, including a storage partition needed for the :ref:`liblwm2m_carrier_readme` library.
* :kconfig:option:`CONFIG_THINGY91_NO_PREDEFINED_LAYOUT` - Enabling this option disables Thingy:91 pre-defined static partitions.
  This allows the application to use a dynamic layout or define a custom static partition layout for the application.
  A debugger is needed to program Thingy:91 for the first time.
  This is an :ref:`experimental <software_maturity>` feature.
