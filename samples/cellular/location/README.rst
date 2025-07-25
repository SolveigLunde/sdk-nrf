.. _location_sample:

Cellular: Location
##################

.. contents::
   :local:
   :depth: 2

The Location sample demonstrates how you can retrieve the location of a device using GNSS, cellular or Wi-Fi® positioning method.
This sample uses the :ref:`lib_location` library.

Requirements
************

The sample supports the following development kits:

.. table-from-sample-yaml::

.. include:: /includes/tfm.txt

See also the requirements in :ref:`lib_location` library document.

.. note::
   .. include:: /includes/nrf_cloud_rest_sample_requirements.txt

Overview
********

The Location sample retrieves the location multiple times to illustrate the different ways of retrieving the location of a device.
Each individual location request has been implemented in a separate function within the sample.
In addition to the :ref:`lib_location` library, this sample uses :ref:`lte_lc_readme` to control the LTE connection.

Configuration
*************

|config|

Additional configuration
========================

Refer to the :ref:`lib_location` library document for configuring the location retrieval behavior, including supported location methods and services.

If you use an external GNSS antenna, add the following configuration:

* :kconfig:option:`CONFIG_MODEM_ANTENNA_GNSS_EXTERNAL` - Selects an external GNSS antenna.

Configuration files
===================

The sample provides predefined configuration files for typical use cases.
The configuration files are in the |sample path| directory.

The following files are available:

* :file:`overlay-nrf700x-wifi-scan-only.conf` - Config overlay for nRF7002 Wi-Fi chip support.
* :file:`overlay-pgps.conf` - Config overlay for P-GPS support.

.. include:: /libraries/modem/nrf_modem_lib/nrf_modem_lib_trace.rst
   :start-after: modem_lib_sending_traces_UART_start
   :end-before: modem_lib_sending_traces_UART_end

Building and running
********************

.. |sample path| replace:: :file:`samples/cellular/location`

.. include:: /includes/build_and_run_ns.txt

nRF91 Series DK with nRF7002 EK Wi-Fi support
=============================================

To build the sample with nRF91 Series DK and nRF7002 EK Wi-Fi support, use the ``-DSHIELD=nrf7002ek``, ``-DEXTRA_CONF_FILE=overlay-nrf700x-wifi-scan-only.conf``, ``-DSB_CONFIG_WIFI_NRF70=y``, and ``-DSB_CONFIG_WIFI_NRF70_SCAN_ONLY=y`` options.
For example:

.. parsed-literal::
   :class: highlight

   west build -p -b *board_target* -- -DSHIELD=nrf7002ek -DEXTRA_CONF_FILE=overlay-nrf700x-wifi-scan-only.conf -DSB_CONFIG_WIFI_NRF70=y -DSB_CONFIG_WIFI_NRF70_SCAN_ONLY=y

|board_target|

See :ref:`cmake_options` for more instructions on how to add these options.

.. note::
   The Thingy:91 X build supports Wi-Fi by default.
   You need not add any overlays.

P-GPS support
=============

To build the Location sample with P-GPS support, use the following commands:

.. parsed-literal::
   :class: highlight

   west build -p -b *board_target* -- -DEXTRA_CONF_FILE=overlay-pgps.conf

|board_target|

See :ref:`cmake_options` for more instructions on how to add this option.

Testing
=======

|test_sample|

#. |connect_kit|
#. |connect_terminal|
#. Observe that the sample prints to the terminal.

Sample output
=============

An example output of the sample:

   .. code-block:: console

      Location sample started

      Connecting to LTE...
      Connected to LTE

      Requesting location with short GNSS timeout to trigger fallback to cellular...
      [00:00:06.481,262] <wrn> location: Timeout occurred
      [00:00:06.487,335] <wrn> location: Failed to acquire location using 'GNSS', trying with 'Cellular' next
      Got location:
      method: cellular
      latitude: 12.887095
      longitude: 55.580397
      accuracy: 1250.0 m
      Google maps URL: https://maps.google.com/?q=12.887095,55.580397

      Requesting location with the default configuration...
      Got location:
      method: GNSS
      latitude: 12.893736
      longitude: 55.575859
      accuracy: 4.4 m
      date: 2021-10-28
      time: 13:36:29.072 UTC
      Google maps URL: https://maps.google.com/?q=12.893736,55.575859

      Requesting location with high GNSS accuracy...
      Got location:
      method: GNSS
      latitude: 12.893755
      longitude: 55.575879
      accuracy: 2.8 m
      date: 2021-10-28
      time: 13:36:32.339 UTC
      Google maps URL: https://maps.google.com/?q=12.893755,55.575879

      Requesting Wi-Fi location with GNSS and cellular fallback...
      Got location:
      method: GNSS
      latitude: 12.893770
      longitude: 55.575884
      accuracy: 4.5 m
      date: 2021-10-28
      time: 13:36:45.895 UTC
      Google maps URL: https://maps.google.com/?q=12.893770,55.575884

      Requesting 30s periodic GNSS location...
      Got location:
      method: GNSS
      latitude: 12.893765
      longitude: 55.575912
      accuracy: 4.4 m
      date: 2021-10-28
      time: 13:36:47.536 UTC
      Google maps URL: https://maps.google.com/?q=12.893765,55.575912

      Got location:
      method: GNSS
      latitude: 12.893892
      longitude: 55.576090
      accuracy: 8.4 m
      date: 2021-10-28
      time: 13:37:17.685 UTC
      Google maps URL: https://maps.google.com/?q=12.893892,55.576090

Dependencies
************

This sample uses the following |NCS| libraries:

* :ref:`lib_location`
* :ref:`lte_lc_readme`
* :ref:`lib_date_time`
* :ref:`lib_at_host`

In addition, it uses the following secure firmware component:

* :ref:`Trusted Firmware-M <ug_tfm>`
