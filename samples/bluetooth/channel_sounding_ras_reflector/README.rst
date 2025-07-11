.. _channel_sounding_ras_reflector:

Bluetooth: Channel Sounding Reflector with Ranging Responder
############################################################

.. contents::
   :local:
   :depth: 2

This sample demonstrates how to use the ranging service to provide ranging data to a client.

Requirements
************

The sample supports the following development kits:

.. table-from-sample-yaml::

The sample also requires a device running a Channel Sounding Initiator with Ranging Requestor to connect to, such as the :ref:`channel_sounding_ras_initiator` sample.

Overview
********

The sample demonstrates a basic Bluetooth® Low Energy Peripheral role functionality that exposes the GATT Ranging Responder Service and configures the Channel Sounding reflector role.
When Channel Sounding Ranging Data is generated by the controller, it will be automatically stored by the Ranging Service, and can be queried at any time by the Ranging Requestor.
The Channel Sounding Ranging Data can then be used by the peer device to perform distance estimation.

User interface
**************

The sample does not require user input and will advertise using the GATT Ranging Service UUID.
The first LED on the development kit will be lit when a connection has been established.

Building and running
********************
.. |sample path| replace:: :file:`samples/bluetooth/channel_sounding_ras_reflector`

.. include:: /includes/build_and_run.txt

Testing
=======

After programming the sample to your development kit, you can test it by connecting to another device programmed with a Channel Sounding Initiator role with Ranging Requestor, such as the :ref:`channel_sounding_ras_initiator` sample.

1. |connect_terminal_specific|
#. Reset the kit.
#. Wait until the advertiser is detected by the Central.
   In the terminal window, check for information similar to the following::

      I: Connected to xx.xx.xx.xx.xx.xx (random) (err 0x00)
      I: CS capability exchange completed.
      I: CS config creation complete. ID: 0
      I: CS security enabled.
      I: CS procedures enabled.

Dependencies
************

This sample uses the following |NCS| libraries:

* :ref:`dk_buttons_and_leds_readme`
* :file:`include/bluetooth/services/ras.h`

This sample uses the following Zephyr libraries:

* :ref:`zephyr:logging_api`:

  * :file:`include/logging/log.h`

* :file:`include/zephyr/types.h`
* :ref:`zephyr:kernel_api`:

  * :file:`include/kernel.h`

* :ref:`zephyr:bluetooth_api`:

* :file:`include/bluetooth/bluetooth.h`
* :file:`include/bluetooth/conn.h`
* :file:`include/bluetooth/uuid.h`
* :file:`include/bluetooth/cs.h`
