.. _ug_nrf52_features:

Features of nRF52 Series
########################

.. contents::
   :local:
   :depth: 2

The nRF52 Series of System-on-Chip (SoC) devices embed an Arm Cortex-M4 processor with Nordic Semiconductor's 2.4 GHz RF transceivers.
All of the nRF52 Series SoCs have support for Bluetooth® 5 features, in addition to multiprotocol capabilities.

To get started with the nRF52 Series, download and run the `Quick Start app`_ in `nRF Connect for Desktop`_.

For additional information, see the following documentation:

* `nRF52 Series`_ for the technical documentation on the nRF52 Series chips and associated kits.
* :ref:`ug_nrf52` for more advanced topics related to the nRF52 Series.
* :ref:`installation` and :ref:`configuration_and_build` documentation to install the |NCS| and learn more about its development environment.

Secure bootloader chain
***********************

nRF52 Series devices support a secure bootloader solution based on the chain of trust concept.

See :ref:`ug_bootloader` for more information and instructions on how to enable one or more bootloaders in your application.

Supported protocols
*******************

The nRF52 Series multiprotocol radio supports Bluetooth Low Energy (LE) including Bluetooth Mesh, proprietary protocols (including Enhanced ShockBurst and Gazell), ANT, Thread, Zigbee, and 802.15.4.
Standard interface protocols like NFC and USB are supported on a range of the devices in the series and with supporting software.

.. note::
   Currently, the |NCS| does not support ANT.

The following sections give pointers on where to start when working with these protocols.

To test the general capabilities of the 2.4 GHz radio transceiver, use the :ref:`radio_test` sample.

Bluetooth Low Energy
====================

When you develop a Bluetooth LE application, you must use the Bluetooth software stack.
This stack is split into two core components: the Bluetooth Host and the Bluetooth LE Controller.

Nordic Semiconductor nRF52 Series devices have full support for the |NCS| Bluetooth stack.
The :ref:`ug_ble_controller` user guide contains more information about the two available Bluetooth LE Controllers and instructions for switching between them.

See :ref:`zephyr:bluetooth` for documentation on the Bluetooth Host and open source Bluetooth LE Controller.
For documentation about the SoftDevice Controller and information on which controller variants support which chips, see :ref:`nrfxlib:softdevice_controller`.

The |NCS| contains a variety of :ref:`ble_samples` that target nRF52 Series devices.
In addition, you can run the :zephyr:code-sample-category:`bluetooth` samples that are included from Zephyr.

For available libraries, see :ref:`lib_bluetooth_services` (|NCS|) and :ref:`zephyr:bluetooth_api` (Zephyr).

Bluetooth Mesh
==============

Bluetooth Mesh is a mesh networking solution based on Bluetooth LE, developed by the Bluetooth Special Interest Group (SIG).
It is optimized for creating large-scale device networks, and implemented according to Bluetooth Mesh Profile Specification v1.0.1 and Bluetooth Mesh Model Specification v1.0.1.

Bluetooth Mesh networking allows one-to-one, one-to-many, and many-to-many communication, using the Bluetooth LE protocol to exchange messages between the mesh nodes in the network.

The |NCS| contains a variety of :ref:`bt_mesh_samples` that target nRF52 Series devices.
In addition, you can run the :zephyr:code-sample-category:`bluetooth` samples that are included from Zephyr.

For available libraries, see :ref:`bt_mesh` (|NCS|) and :ref:`zephyr:bluetooth_mesh` (Zephyr).
See the :ref:`ug_bt_mesh` user guide for information about how to use the supplied libraries and work with Bluetooth Mesh.

Enhanced ShockBurst
===================

.. include:: /protocols/esb/index.rst
   :start-after: esb_intro_start
   :end-before: esb_intro_end

See the :ref:`ug_esb` user guide for information about how to work with Enhanced ShockBurst.
To start developing, check out the :ref:`esb_ptx` and :ref:`esb_prx` samples.

Gazell
======

.. include:: /protocols/gazell/index.rst
   :start-after: gz_intro_start
   :end-before: gz_intro_end

See the :ref:`ug_gzll` user guide and the :ref:`ug_gzp` user guide for information about how to work with Gazell.
To start developing, check out the :ref:`gazell_samples`.

Matter
======

.. include:: /protocols/matter/index.rst
   :start-after: matter_intro_start
   :end-before: matter_intro_end

See the :ref:`ug_matter` user guide for information about how to work with Matter applications.
To start developing, check out :ref:`matter_samples`.

Near Field Communication
========================

Near Field Communication (NFC) is a technology for wireless transfer of small amounts of data between two devices that are in close proximity.
The range of NFC is typically <10 cm.

|NCS| provides two protocol stacks for developing NFC applications: Type 2 Tag and Type 4 Tag.
These stacks are provided in binary format in the `sdk-nrfxlib`_ repository.
See :ref:`nrfxlib:nfc` for documentation about the NFC stacks, and :ref:`ug_nfc` for general information.

The NFC stack requires the NFCT driver for nRF52 devices, which is available as part of `nrfx`_.
The nrfx repository is included in the |NCS| as a module of the Zephyr repository.

See :ref:`nfc_samples` and :ref:`lib_nfc` for lists of samples and libraries that the |NCS| provides.

USB
===

The |NCS| contains a USB device stack for the USB 2.0 Full Speed peripheral that is available on a number of the nRF52 devices.
You can find the implementation in the Zephyr repository.
See :ref:`zephyr:usb_api` for documentation and :zephyr:code-sample-category:`usb` for a list of available samples.

The USB stack requires the USBD driver for nRF52 devices, which is available as part of `nrfx`_.
The nrfx repository is included in the |NCS| as a module of the Zephyr repository.

Thread
======

.. include:: /protocols/thread/index.rst
   :start-after: thread_intro_start
   :end-before: thread_intro_end

See the :ref:`ug_thread` user guide on how to work with Thread.
To start developing, refer to the :ref:`openthread_samples`.

Zigbee
======

Zigbee is a portable, low-power software networking protocol that provides connectivity over a mesh network based on the IEEE 802.15.4 radio protocol.
It also defines an application layer that provides interoperability among all Zigbee devices.

For more information, see the :ref:`Zigbee protocol<ug_zigbee>` support.

Multiprotocol support
*********************

The nRF52 Series devices support running another protocol in parallel with the SoftDevice Controller.
See the :ref:`ug_multiprotocol_support` user guide on how to enable multiprotocol support for Thread or Zigbee in combination with Bluetooth.

The :ref:`nrfxlib:mpsl` library provides services for multiprotocol applications.

Remote observability using Memfault
***********************************

The |NCS| bundles support for remotely monitoring and debugging device fleets.
This support enables quicker identification and triage of issues in the field and optimizes connection quality and battery life for global deployments.
The collection system has been optimized to work in intermittent connectivity environments and has extremely low overhead.

The |NCS| includes out-of-the-box metrics collected for monitoring Bluetooth connectivity, such as connection time and Bluetooth thread stack usage on nRF52 Series SoCs as well as a GATT profile and example apps for easily sending the data through a mobile phone gateway.

See the :ref:`ug_memfault` page for more information on how to enable Memfault in your |NCS| project on an nRF52 Series device and visualize the data across the fleet and by device.
