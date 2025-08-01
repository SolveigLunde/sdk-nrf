.. _timeslot_sample:

MPSL timeslot
#############

.. contents::
   :local:
   :depth: 2

This sample demonstrates how to use :ref:`nrfxlib:mpsl` and basic MPSL Timeslot functionality.

Requirements
************

The sample supports any one of the following development kits:

.. table-from-sample-yaml::

.. note::
   For the nRF5340 DK, this sample is only supported on the network core (``nrf5340dk/nrf5340/cpunet``), and the :ref:`nrf5340_empty_app_core` sample must be programmed to the application core.
   For the nRF54H20 DK, this sample is only supported on the radio core (``nrf54h20dk/nrf4h20/cpurad``).

Overview
********

The sample opens a timeslot session and starts requesting timeslots when a key is pressed in the terminal.

* If ``a`` is pressed, the callback for the first timeslot requests a new timeslot.
* If ``b`` is pressed, the callback for the first timeslot ends the timeslot.

The first timeslot is always of type ``earliest``.
Any following timeslots are of type ``normal``.
In each timeslot callback, the signal type of the callback is posted to a message queue.
Upon reception of the timeslot start signal, ``mpsl timer0`` is configured to be triggered before the timeslot ends.
A separate thread reads the message queue and prints the timeslot signal type.
The timeslot session is closed when any key is pressed in the terminal.

.. note::
   For the nRF52 and nRF53 Series ``mpsl_timer0`` is the ``timer0`` instance.
   For the nRF54L Series ``mpsl_timer0`` is the ``timer10`` instance.
   For the nRF54H Series ``mpsl_timer0`` is the ``timer020`` instance.

.. note::
   For the nRF54H20, if you want to use ``MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE`` then you must request the HFXO earlier using the :c:func:`mpsl_clock_hfclk_request` function.

Building and running
********************

.. |sample path| replace:: :file:`samples/mpsl/timeslot`

.. include:: /includes/build_and_run.txt

Testing
=======

After programming the sample to your development kit, complete the following steps to test it:

1. |connect_terminal|
#. Follow the instructions in the terminal to open a session and start requesting timeslots.
   The terminal then prints the signal type for each timeslot callback:

   * If you press ``a``, the timeslot callback requests a new timeslot.
     Observe that ``Timeslot start`` and ``Timer0 signal`` are printed until the session is closed.
   * If you press ``b``, the timeslot callback ends the timeslot.
     Observe that ``Timeslot start`` and ``Timer0 signal`` are printed, followed by ``Session idle``.

Dependencies
************

This sample uses the following `sdk-nrfxlib`_ libraries:

* :ref:`nrfxlib:mpsl`

In addition, it uses the following Zephyr libraries:

* :file:`include/console.h`
* :ref:`zephyr:kernel_api`:

  * :file:`include/kernel.h`
  * :file:`include/irq.h`

* :file:`include/sys/printk.h`
* :file:`include/zephyr/types.h`
