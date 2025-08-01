.. _thread_power_consumption:

OpenThread power consumption
############################

.. contents::
   :local:
   :depth: 2

This page provides information about the amount of power used by Sleepy End Devices (SEDs) and Synchronized Sleepy End Devices (SSEDs) that use the OpenThread stack.

.. _thread_power_consumption_methodology:

Measurements methodology
************************

The measurement setup consists of the following:

* An nRF52840 DK board used as a Thread leader.
* A DUT board used as a Thread child.
* A `Power Profiler Kit II (PPK2)`_ attached to the DUT according to the instructions in its Quick start guide.

The leader board is flashed with the regular :ref:`Thread CLI sample <ot_cli_sample>` firmware.

.. code-block::

   cd ncs/nrf/samples/openthread/cli/
   west build -b nrf52840dk/nrf52840 -- -Dcli_SNIPPET="ci;logging"

The DUT board is flashed with the :ref:`Thread CLI sample low power mode <ot_cli_sample_low_power>` firmware.
In the build command below, replace *board_target* with the board target name of the DUT.

.. parsed-literal::
   :class: highlight

   cd ncs/nrf/samples/openthread/cli/
   west build -b *board_target* -- -Dcli_SNIPPET="ci;low_power"


After the Thread network is enabled on the leader, the child is configured with the desired parameters.
Once attached to the network, the child device will switch to sleep mode (disabling all peripherals except RTC) and wake up according to its configuration:

* For the SED case, it will regularly send MAC Data Poll Commands.
* For the SSED case, it will regularly switch to radio receive mode.

The PPK2 is used to collect power consumption data during at least 60 seconds.
During this time no data frames are being sent from the leader to the child.
Afterwards, the CSV data is parsed with a script to produce the below results.

.. _thread_power_consumption_data:

Power consumption data
**********************

The following tables show the power consumption measured with the configuration used.

.. tabs::

   .. group-tab:: Thread 1.4 SED

      .. table:: Configuration

         +--------------------------+---------------------+--------------------------+----------------------------+
         | Parameter                | nrf52840dk/nrf52840 | nrf5340dk/nrf5340/cpuapp | nrf54l15dk/nrf54l15/cpuapp |
         +==========================+=====================+==========================+============================+
         | Board revision           |              v3.0.2 |                   v2.0.2 |                     v0.8.1 |
         +--------------------------+---------------------+--------------------------+----------------------------+
         | Supply voltage [V]       |                 3.0 |                      3.0 |                       3.0  |
         +--------------------------+---------------------+--------------------------+----------------------------+
         | Transmission power [dBm] |                   0 |                        0 |                         0  |
         +--------------------------+---------------------+--------------------------+----------------------------+
         | Polling period [ms]      |                1000 |                     1000 |                      1000  |
         +--------------------------+---------------------+--------------------------+----------------------------+

      .. table:: Power consumption

        +-------------------------------+-----------------------+----------------------------+----------------------------+
        | Parameter                     |   nrf52840dk/nrf52840 |   nrf5340dk/nrf5340/cpuapp | nrf54l15dk/nrf54l15/cpuapp |
        +===============================+=======================+============================+============================+
        | Total charge per minute [μC]  |               1161.43 |                    1134.43 |                    829.99  |
        +-------------------------------+-----------------------+----------------------------+----------------------------+
        | Average data poll charge [μC] |                 15.81 |                      16.15 |                     10.82  |
        +-------------------------------+-----------------------+----------------------------+----------------------------+
        | Average sleep current [μA]    |                  3.56 |                       2.79 |                      2.75  |
        +-------------------------------+-----------------------+----------------------------+----------------------------+


   .. group-tab:: Thread 1.4 SSED

      .. table:: Configuration

         +--------------------------------+---------------------+--------------------------+----------------------------+
         | Parameter                      | nrf52840dk/nrf52840 | nrf5340dk/nrf5340/cpuapp | nrf54l15dk/nrf54l15/cpuapp |
         +================================+=====================+==========================+============================+
         | Board revision                 |              v3.0.2 |                   v2.0.2 |                     v0.8.1 |
         +--------------------------------+---------------------+--------------------------+----------------------------+
         | Supply voltage [V]             |                 3.0 |                      3.0 |                       3.0  |
         +--------------------------------+---------------------+--------------------------+----------------------------+
         | Transmission power [dBm]       |                   0 |                        0 |                         0  |
         +--------------------------------+---------------------+--------------------------+----------------------------+
         | CSL period [ms]                |                1000 |                     1000 |                      1000  |
         +--------------------------------+---------------------+--------------------------+----------------------------+
         | CSL timeout [s]                |                  20 |                       20 |                        20  |
         +--------------------------------+---------------------+--------------------------+----------------------------+
         | Parent's CSL accuracy [ppm]    |                 ±20 |                      ±20 |                       ±20  |
         +--------------------------------+---------------------+--------------------------+----------------------------+
         | Parent's CSL uncertainty [μs]  |                ±120 |                     ±120 |                      ±120  |
         +--------------------------------+---------------------+--------------------------+----------------------------+

      .. table:: Power consumption

         +---------------------------------+-----------------------+----------------------------+----------------------------+
         | Parameter                       |   nrf52840dk/nrf52840 |   nrf5340dk/nrf5340/cpuapp | nrf54l15dk/nrf54l15/cpuapp |
         +=================================+=======================+============================+============================+
         | Total charge per minute [μC]    |               1072.57 |                    1046.71 |                    929.19  |
         +---------------------------------+-----------------------+----------------------------+----------------------------+
         | Average CSL receive charge [μC] |                 13.21 |                      13.68 |                     11.66  |
         +---------------------------------+-----------------------+----------------------------+----------------------------+
         | Average data poll charge [μC]   |                 21.75 |                      21.61 |                     14.78  |
         +---------------------------------+-----------------------+----------------------------+----------------------------+
         | Average sleep current [μA]      |                  3.58 |                       2.70 |                      2.75  |
         +---------------------------------+-----------------------+----------------------------+----------------------------+
