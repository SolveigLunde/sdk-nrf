.. _download_sample:

Download
########

.. contents::
   :local:
   :depth: 2

The Download sample demonstrates how to download a file from an HTTP or a CoAP server, with optional TLS or DTLS.
It uses the :ref:`lib_downloader` library.

.. |wifi| replace:: Wi-Fi®

.. include:: /includes/net_connection_manager.txt

Requirements
************

The sample supports the following development kits:

.. table-from-sample-yaml::

.. include:: /includes/tfm.txt

Overview
********

The sample first initializes the :ref:`nrfxlib:nrf_modem` and AT communications.
Next, if the :ref:`CONFIG_SAMPLE_PROVISION_CERT <CONFIG_SAMPLE_PROVISION_CERT>` is set, it provisions a certificate to the modem using the :ref:`modem_key_mgmt` library if the :ref:`CONFIG_SAMPLE_SECURE_SOCKET <CONFIG_SAMPLE_SECURE_SOCKET>` option is set.
When using an nRF91 Series device, the provisioning of the certificates must be done before connecting to the LTE network since the certificates can only be provisioned when the device is not connected.
The certificate file name and security tag can be configured using the :ref:`CONFIG_SAMPLE_SEC_TAG <CONFIG_SAMPLE_SEC_TAG>` and the :ref:`CONFIG_SAMPLE_CERT_FILE <CONFIG_SAMPLE_CERT_FILE>` options, respectively.

The sample then performs the following actions:

1. Establishes a connection to the network
#. Optionally sets up the secure socket options
#. Uses the :ref:`lib_downloader` library to download a file from an HTTP server.


Downloading from a CoAP server
==============================

To enable CoAP block-wise transfer, it is necessary to enable :ref:`Zephyr's CoAP stack <zephyr:coap_sock_interface>` using the :kconfig:option:`CONFIG_COAP` option.

Using TLS and DTLS
==================

By default, the :ref:`CONFIG_SAMPLE_PROVISION_CERT <CONFIG_SAMPLE_PROVISION_CERT>` option is set, which means that the sample provisions the certificate found in the :file:`samples/net/download/cert` folder.
The certificate file name is indicated by the :ref:`CONFIG_SAMPLE_CERT_FILE <CONFIG_SAMPLE_CERT_FILE>` option.
This certificate will work for the default test files.
If you are using a custom download test file, you must provision the correct certificate for the servers from which the certificates will be downloaded.

|hex_format|

See :ref:`cert_dwload` for more information.

Configuration
*************

|config|

Configuration options
=====================

Check and configure the following configuration options for the sample:

.. _CONFIG_SAMPLE_SECURE_SOCKET:

CONFIG_SAMPLE_SECURE_SOCKET - Secure socket configuration
   If enabled, downloading is done using a secure socket over TLS or DTLS.

.. _CONFIG_SAMPLE_SEC_TAG:

CONFIG_SAMPLE_SEC_TAG - Security tag configuration
   This option configures the security tag.

.. _CONFIG_SAMPLE_PROVISION_CERT:

CONFIG_SAMPLE_PROVISION_CERT - Root CA Certificate provision
   If enabled, this option provisions the certificate to the modem.

.. _CONFIG_SAMPLE_CERT_FILE:

CONFIG_SAMPLE_CERT_FILE - Certificate file name configuration
   This option sets the certificate file name.

.. _CONFIG_SAMPLE_COMPUTE_HASH:

CONFIG_SAMPLE_COMPUTE_HASH - Hash compute configuration
   If enabled, this option computes the SHA256 hash of the downloaded file.

.. _CONFIG_SAMPLE_COMPARE_HASH:

CONFIG_SAMPLE_COMPARE_HASH - Hash compare configuration
   If enabled, this option compares the hash against the SHA256 hash set by :ref:`CONFIG_SAMPLE_SHA256_HASH <CONFIG_SAMPLE_SHA256_HASH>` for a match.

.. _CONFIG_SAMPLE_SHA256_HASH:

CONFIG_SAMPLE_SHA256_HASH - Hash configuration
   This option sets the SHA256 hash to be compared with :ref:`CONFIG_SAMPLE_COMPUTE_HASH <CONFIG_SAMPLE_COMPUTE_HASH>`.

.. include:: /includes/wifi_credentials_shell.txt

.. include:: /includes/wifi_credentials_static.txt

.. include:: /libraries/modem/nrf_modem_lib/nrf_modem_lib_trace.rst
   :start-after: modem_lib_sending_traces_UART_start
   :end-before: modem_lib_sending_traces_UART_end

Building and running
********************

.. |sample path| replace:: :file:`samples/net/download`

.. include:: /includes/build_and_run_ns.txt


Testing
=======

After programming the sample to your development kit, test it by performing the following steps:

1. |connect_kit|
#. Power on or reset the kit.
#. |connect_terminal_ANSI|
#. Observe that the sample starts, provisions certificates, and starts to download.
#. Observe that the progress bar fills up as the download progresses.
#. Observe that the sample displays the message "Download completed" on the terminal when the download completes.

Sample output
=============

The following output is logged on the terminal when the sample downloads a file from an HTTPS server:

.. code-block:: console

   Download client sample started
   Provisioning certificate
   Connecting to network
   IP Up
   Network connected
   Downloading https://nrfconnectsdk.s3.eu-central-1.amazonaws.com/sample-img-100kb.png
   [ 100% ] |==================================================| (102923/102923 bytes)
   Download completed in 13679 ms @ 7524 bytes per sec, total 102923 bytes
   IP down
   Disconnected from network
   Socket closed
   Bye

Dependencies
************

This sample uses the following |NCS| libraries when using an nRF91 Series device:

* :ref:`modem_key_mgmt`
* :ref:`nrf_modem_lib_readme`

It uses the following `sdk-nrfxlib`_ library:

* :ref:`nrfxlib:nrf_modem`

In addition, it uses the following secure firmware component:

* :ref:`Trusted Firmware-M <ug_tfm>`
