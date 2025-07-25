.. _ei_data_forwarder_sample:

Edge Impulse: Data forwarder
############################

.. contents::
   :local:
   :depth: 2

The Edge Impulse data forwarder sample demonstrates the usage of `Edge Impulse's data forwarder`_ to provide sensor data to `Edge Impulse studio`_ when :ref:`integrating Edge Impulse with the nRF Connect SDK <ug_edge_impulse>`.
The sample forwards simulated accelerometer data generated by the :ref:`sensor_sim`.

Requirements
************

The sample supports the following development kits:

.. table-from-sample-yaml::

.. include:: /includes/tfm.txt

Overview
********

The sample periodically performs the following operations:

* Reads data from sensor.
  The sample reads simulated acceleration measurements for X, Y, and Z axes.
* Forwards the data through UART using the protocol specified by `Edge Impulse's data forwarder`_.

.. note::
   The sample uses UART only to forward the sensor data.
   For this reason, sample logs are provided through RTT, unlike in other samples.

See `Edge Impulse`_ website for more information about the Edge Impulse platform.

Configuration
*************

|config|

Setup
=====

Before running the sample, you must complete the following steps:

1. Prepare your own project using `Edge Impulse studio`_ external web tool.
   See :ref:`ug_edge_impulse` for more information about starting using the tool.
2. Follow the `Edge Impulse CLI installation guide`_ to install Edge Impulse command line tools.
   These tools are needed because the ``edge-impulse-data-forwarder`` is used to forward the data received from the device through UART to Edge Impulse studio.

Building and running
********************

.. |sample path| replace:: :file:`samples/edge_impulse/data_forwarder`

.. include:: /includes/build_and_run_ns.txt

Testing
=======

After programming the sample to your development kit, test it by performing the following steps:

1. Run the ``edge-impulse-data-forwarder`` Edge Impulse command line tool.
   The tool connects the device to your Edge Impulse project.
   See `Edge Impulse's data forwarder`_ documentation for a guide.
#. Trigger sampling data from the device using Edge Impulse studio:

   a. Go to the :guilabel:`Data acquisition` tab.
   #. In the **Record new data** panel, set the desired values and click :guilabel:`Start sampling`.

      .. figure:: ../../../doc/nrf/images/ei_data_acquisition.png
         :scale: 50 %
         :alt: Sampling under Data acquisition in Edge Impulse studio

         Sampling under Data acquisition in Edge Impulse studio

   #. Observe the received sample data on the raw data graph under the panel.
      For the default sample configuration, you should observe sine waves.

      .. figure:: ../../../doc/nrf/images/ei_start_sampling.png
         :scale: 50 %
         :alt: Sampling example

         Sampling example

Dependencies
************

This sample uses the following |NCS| drivers:

* :ref:`sensor_sim`

In addition, it uses the following Zephyr drivers:

* :ref:`zephyr:uart_api`

The sample also uses the following secure firmware component:

* :ref:`Trusted Firmware-M <ug_tfm>`
