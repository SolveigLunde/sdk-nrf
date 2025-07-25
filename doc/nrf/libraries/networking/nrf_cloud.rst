.. _lib_nrf_cloud:

nRF Cloud
#########

.. contents::
   :local:
   :depth: 2

The nRF Cloud library enables applications to connect to Nordic Semiconductor's `nRF Cloud`_.
It abstracts and hides the details of the transport and the encoding scheme that is used for the payload and provides a simplified API interface for sending data to the cloud.
The current implementation supports the following technologies:

* GNSS, TEMP, and other application-specific sensor data
* Cellular and Wi-Fi® location data
* TLS-secured MQTT, TLS-secured REST, or DTLS-secured CoAP as the communication protocol
* JSON as the data format for MQTT and REST
* CBOR and JSON as the data format for CoAP

.. note::
   The remainder of this document describes the nRF Cloud library's MQTT connectivity support.
   See the :ref:`lib_nrf_cloud_rest` library and the :ref:`lib_nrf_cloud_coap` library for other connectivity options.

.. _lib_nrf_cloud_init:

Initializing
************

Before using any other APIs of the module, set the :kconfig:option:`CONFIG_NRF_CLOUD` Kconfig option and call the :c:func:`nrf_cloud_init` function in the application.
If this call fails, the application must not use any functions of the module.

.. note::
   Initialize the module before starting any timers, sensor drivers, or communication on the link.

.. _lib_nrf_cloud_connect:

Connecting
**********
The application can use the :c:func:`nrf_cloud_connect` function to connect to the cloud.
This API triggers a series of events and actions in the system.
If the API fails, the application must retry to connect.

If the :kconfig:option:`CONFIG_NRF_CLOUD_CONNECTION_POLL_THREAD` Kconfig option is not enabled, the application should monitor the connection socket.
The :c:func:`nrf_cloud_connect` function blocks and returns success when the MQTT connection to the cloud completes.

If the :kconfig:option:`CONFIG_NRF_CLOUD_CONNECTION_POLL_THREAD` Kconfig option is enabled, an nRF Cloud library thread monitors the connection socket.
The :c:func:`nrf_cloud_connect` function does not block and returns success if the connection monitoring thread has started.

When the :kconfig:option:`CONFIG_NRF_CLOUD_CONNECTION_POLL_THREAD` Kconfig option is enabled, an additional event, :c:enum:`NRF_CLOUD_EVT_TRANSPORT_CONNECTING`, is sent to the application.
To adjust the stack size of the connection monitoring thread, set the :kconfig:option:`CONFIG_NRF_CLOUD_CONNECTION_POLL_THREAD_STACK_SIZE` Kconfig option.
The :c:enum:`NRF_CLOUD_EVT_TRANSPORT_CONNECT_ERROR` event is sent if an error occurs while the transport connection is being established.
The status field of the :c:struct:`nrf_cloud_evt` structure contains the reason for the error that is defined by :c:enum:`nrf_cloud_connect_result`.
The :c:enumerator:`NRF_CLOUD_EVT_TRANSPORT_DISCONNECTED` event also contains additional information in the status field that is defined by :c:enum:`nrf_cloud_disconnect_status`.

First, the library tries to establish the transport for communicating with the cloud.
This procedure involves a TLS handshake that might take up to three seconds.
The API blocks for the duration of the handshake.

The cloud uses the certificates of the device for authentication.
See :ref:`nrf9160_ug_updating_cloud_certificate` and the :ref:`modem_key_mgmt` library for more information on modem credentials.
The device ID is also the MQTT client ID.
There are multiple configuration options for the device or client ID.
See :ref:`configuration_device_id` for more information.

As the next step, the API subscribes to an MQTT topic to start receiving requests from the cloud.

Associating
***********
This section applies to devices onboarding using JITP.

Every time nRF Cloud starts a communication session with a device, it verifies whether the device is uniquely associated with a user.
If not, the user association procedure is triggered.

The following message sequence chart shows the flow of events and the expected application responses to each event during the user association procedure:

.. msc::
   hscale = "1.3";
   Module,Application;
   Module<<Application      [label="nrf_cloud_connect() returns successfully"];
   Module>>Application      [label="NRF_CLOUD_EVT_TRANSPORT_CONNECTED"];
   Module>>Application      [label="NRF_CLOUD_EVT_USER_ASSOCIATION_REQUEST"];
    ---                     [label="Add the device to nRF Cloud account"];
   Module>>Application      [label="NRF_CLOUD_EVT_USER_ASSOCIATED"];
   Module<<Application      [label="nrf_cloud_disconnect() returns successfully"];
   Module>>Application      [label="NRF_CLOUD_EVT_TRANSPORT_DISCONNECTED"];
   Module<<Application      [label="nrf_cloud_connect() returns successfully"];
   Module>>Application      [label="NRF_CLOUD_EVT_TRANSPORT_CONNECTED"];
   Module>>Application      [label="NRF_CLOUD_EVT_USER_ASSOCIATED"];
   Module>>Application      [label="NRF_CLOUD_EVT_READY"];

The chart shows the sequence of successful user association of an unassociated device.

.. note::

   Currently, nRF Cloud requires that communication is re-established to update the device's permission to send user data.
   The application must disconnect using the :c:func:`nrf_cloud_disconnect` function and then reconnect using :c:func:`nrf_cloud_connect`.

When the device is successfully associated with a user on the cloud, subsequent connections to the cloud (also across power cycles) occur in the following sequence:

.. msc::
   hscale = "1.3";
   Module,Application;
   Module<<Application      [label="nrf_cloud_connect() returns successfully"];
   Module>>Application      [label="NRF_CLOUD_EVT_TRANSPORT_CONNECTED"];
   Module>>Application      [label="NRF_CLOUD_EVT_USER_ASSOCIATED"];
   Module>>Application      [label="NRF_CLOUD_EVT_READY"];

After receiving the :c:enumerator:`NRF_CLOUD_EVT_READY` event, the application can start sending sensor data to the cloud.

.. _nrf_cloud_onboarding:

nRF Cloud onboarding options
============================

You can add a device to an nRF Cloud account in the following three ways:

* Using the :ref:`lib_nrf_provisioning` service and `nRF Cloud Auto-onboarding`_: once the process completes, the device will be listed in your account.
* Using JITP with factory-installed certificates for Nordic development kits and Thingys: provide the device ID and PIN to nRF Cloud as indicated on the sticker.
* Using scripted provisioning and onboarding: upload the :file:`onboard.csv` file to nRF Cloud's **Bulk Onboard** screen or use the REST API.

See: the `nRF Cloud Provisioning`_ documentation and the :ref:`nrf_cloud_multi_service_provisioning_onboarding` section of the :ref:`nrf_cloud_multi_service` sample documentation for more information.

.. _configuration_device_id:

Configuration options for device ID
===================================

* :kconfig:option:`CONFIG_NRF_CLOUD_CLIENT_ID_SRC_IMEI` - If you enable this option, the ID is automatically generated using a prefix and the modem's IMEI (``<prefix><IMEI>``).
  This is the default.
  You can configure the prefix by using :kconfig:option:`CONFIG_NRF_CLOUD_CLIENT_ID_PREFIX`.
  The default format of the prefix is ``nrf-`` and it is valid only for Nordic devices such as Thingy:91, Thingy:91 X, or an nRF91 Series DK.
  For custom hardware, use a prefix other than ``nrf-`` by modifying :kconfig:option:`CONFIG_NRF_CLOUD_CLIENT_ID_PREFIX`.

* :kconfig:option:`CONFIG_NRF_CLOUD_CLIENT_ID_SRC_INTERNAL_UUID` - If you enable this option, the ID is automatically generated using the modem's 128-bit internal UUID, which results in a 36 character string of hexadecimal values in the 8-4-4-4-12 UUID format.

  * This option requires modem firmware v1.3.0 or higher.
  * This option is required when using `auto-onboarding <nRF Cloud Auto-onboarding_>`_.
  * This option only takes effect if the :kconfig:option:`CONFIG_MODEM_JWT` Kconfig option is also enabled.
    If the :kconfig:option:`CONFIG_MODEM_JWT` Kconfig option is not set to ``y``, the default :kconfig:option:`CONFIG_NRF_CLOUD_CLIENT_ID_SRC_IMEI` Kconfig option will be selected instead.

* :kconfig:option:`CONFIG_NRF_CLOUD_CLIENT_ID_SRC_COMPILE_TIME` - If you enable this option, the ID is set at compile time using the value specified by :kconfig:option:`CONFIG_NRF_CLOUD_CLIENT_ID`.

* :kconfig:option:`CONFIG_NRF_CLOUD_CLIENT_ID_SRC_HW_ID` - If you enable this option, the ID is automatically generated using a unique hardware ID (for example, a MAC address).
  You can choose the required hardware ID using the ``HW_ID_LIBRARY_SOURCE`` Kconfig choice.
  See the :ref:`lib_hw_id` library documentation for details.

* :kconfig:option:`CONFIG_NRF_CLOUD_CLIENT_ID_SRC_RUNTIME` - If you enable this option, the ID is set at runtime.
  If the nRF Cloud library is used directly, set the NULL-terminated ID string in the :c:struct:`nrf_cloud_init_param` structure when calling the :c:func:`nrf_cloud_init` function.

.. _lib_nrf_cloud_fota:

Firmware over-the-air (FOTA) updates
************************************

The nRF Cloud library supports FOTA updates for your nRF91 Series device.
The :kconfig:option:`CONFIG_NRF_CLOUD_FOTA` Kconfig option is enabled by default when :kconfig:option:`CONFIG_NRF_CLOUD_MQTT` is set.
This enables FOTA functionality in the application.
FOTA support for applications using CoAP or REST is enabled with the :kconfig:option:`CONFIG_NRF_CLOUD_FOTA_POLL` Kconfig option.

nRF Cloud FOTA enables the following additional features and libraries:

* :kconfig:option:`CONFIG_FOTA_DOWNLOAD` enables :ref:`lib_fota_download`
* :kconfig:option:`CONFIG_DFU_TARGET` enables :ref:`lib_dfu_target`
* :kconfig:option:`CONFIG_DOWNLOADER` enables :ref:`lib_downloader`
* :kconfig:option:`CONFIG_FOTA_DOWNLOAD_PROGRESS_EVT`
* :kconfig:option:`CONFIG_FOTA_PROGRESS_EVT_INCREMENT`
* :kconfig:option:`CONFIG_REBOOT`
* :kconfig:option:`CONFIG_CJSON_LIB`
* :kconfig:option:`CONFIG_SETTINGS`

For FOTA updates to work, the device must provide the information about the supported FOTA types to nRF Cloud.
The device passes this information by writing a ``fota_v2`` field containing an array of FOTA types into the ``serviceInfo`` field in the device's shadow.
The :c:func:`nrf_cloud_service_info_json_encode` function can be used to generate the proper JSON data to enable FOTA.
Additionally, the :c:func:`nrf_cloud_shadow_device_status_update` function can be used to generate the JSON data and perform the shadow update.

Following are the supported FOTA types:

* ``"APP"`` - Updates the application.
* ``"BOOT"`` - Updates the :ref:`upgradable_bootloader`.
* ``"MDM_FULL"`` - :ref:`Full modem FOTA <nrf_modem_bootloader>` updates the entire modem firmware image.
  Full modem updates require |external_flash_size| of available space.
  For an nRF91 Series device, a full modem firmware image is approximately 2 MB.
  Consider the power and network costs before deploying full modem FOTA updates.
* ``"MODEM"`` - :ref:`Delta modem FOTA <nrf_modem_delta_dfu>` applies incremental changes between specific versions of the modem firmware.
  Delta modem updates are much smaller in size and do not require external memory.
* ``"SMP"`` - Updates an auxiliary device's firmware using the :ref:`Simple Management Protocol <zephyr:device_mgmt>`.

For example, a device that supports all the FOTA types writes the following data into the device shadow:

.. code-block::

   {
   "state": {
      "reported": {
         "device": {
            "serviceInfo": {
               "fota_v2": [
               "APP",
               "MODEM",
               "MDM_FULL",
               "BOOT"
               ]
   }}}}}

You can initiate FOTA updates through `nRF Cloud`_ or by using the `nRF Cloud REST API (v1)`_.
If the :kconfig:option:`CONFIG_NRF_CLOUD_FOTA` Kconfig option is enabled, FOTA update job information is requested by the device after the MQTT connection to nRF Cloud is completed.
The :kconfig:option:`CONFIG_NRF_CLOUD_FOTA_AUTO_START_JOB` Kconfig option controls how FOTA jobs are started on the device.

* If enabled, the nRF Cloud library starts the FOTA update job immediately upon receipt of the FOTA update job information from nRF Cloud.
  If the job is successfully started, the library sends the :c:enumerator:`NRF_CLOUD_EVT_FOTA_START` event to the application.
* If disabled, the :c:enumerator:`NRF_CLOUD_EVT_FOTA_JOB_AVAILABLE` event is sent to the application.
  When the application is ready to start the FOTA update job it must call the :c:func:`nrf_cloud_fota_job_start` function.

The FOTA update is in progress until the application receives either the :c:enumerator:`NRF_CLOUD_EVT_FOTA_DONE` or :c:enumerator:`NRF_CLOUD_EVT_FOTA_ERROR` event.
When receiving the :c:enumerator:`NRF_CLOUD_EVT_FOTA_DONE` event, the application must perform any necessary cleanup and reboot the device to complete the update.
The message payload of the :c:enumerator:`NRF_CLOUD_EVT_FOTA_DONE` event contains the :c:enum:`nrf_cloud_fota_type` value.
If the value equals :c:enumerator:`NRF_CLOUD_FOTA_MODEM_DELTA`, the application can optionally avoid a reboot by reinitializing the modem library and then calling the :c:func:`nrf_cloud_modem_fota_completed` function.

See `nRF Cloud FOTA`_ for details on the FOTA service in nRF Cloud.
See `nRF Cloud MQTT FOTA`_ for MQTT-specific FOTA details such as topics and payload formats.

Building FOTA images
====================
The build system will create the files :file:`dfu_application.zip` or :file:`dfu_mcuboot.zip` (or both) for a properly configured application.
See :ref:`app_build_output_files` for more information about FOTA zip files.

When you use the files :file:`dfu_application.zip` or :file:`dfu_mcuboot.zip` to create an update bundle, the `nRF Cloud`_ UI populates the ``Name`` and ``Version`` fields from the :file:`manifest.json` file contained within.
However, you are free to change them as needed.
The UI populates the ``Version`` field from only the |NCS| version field in the :file:`manifest.json` file.

Alternatively, you can use the :file:`app_update.bin` file to create an update bundle, but you need to enter the ``Name`` and ``Version`` fields manually.
See `nRF Cloud Getting Started FOTA documentation`_ to learn how to create an update bundle.

Modem firmware is controlled by Nordic Semiconductor.
A user cannot build or upload modem firmware images.
Modem FOTA update bundles (full and delta) are automatically uploaded to nRF Cloud and are available to all users.

.. _lib_nrf_cloud_data:

Sending sensor data
*******************
The library offers two functions, :c:func:`nrf_cloud_sensor_data_send` and :c:func:`nrf_cloud_sensor_data_stream` (lowest QoS), for sending sensor data to the cloud.

.. _lib_nrf_cloud_unlink:

Removing the link between device and user
*****************************************

If you want to remove the link between a device and an nRF Cloud account, you must do this from nRF Cloud.
A device cannot remove itself from an nRF Cloud account.

.. _lib_nrf_cloud_location_services:

Location services
*****************

`nRF Cloud`_ offers location services that allow you to obtain the location of your device.
The following enhancements to this library can be used to interact with `nRF Cloud Location Services <nRF Cloud Location Services documentation_>`_:

* Assisted GNSS - :ref:`lib_nrf_cloud_agnss`
* Predicted GPS - :ref:`lib_nrf_cloud_pgps`
* Cellular Positioning - :ref:`lib_nrf_cloud_cell_pos`
* nRF Cloud REST  - :ref:`lib_nrf_cloud_rest`

.. _nrf_cloud_api:

API documentation
*****************

| Header file: :file:`include/net/nrf_cloud.h`
| Source files: :file:`subsys/net/lib/nrf_cloud/src/`

.. doxygengroup:: nrf_cloud

nRF Cloud codec documentation
*****************************

| Header file: :file:`include/net/nrf_cloud_codec.h`

.. doxygengroup:: nrf_cloud_codec

nRF Cloud common definitions
****************************

| Header file: :file:`include/net/nrf_cloud_defs.h`

.. doxygengroup:: nrf_cloud_defs

nRF Cloud FOTA poll for REST and CoAP
****************************************

| Header file: :file:`include/net/nrf_cloud_fota_poll.h`

.. doxygengroup:: nrf_cloud_fota_poll
