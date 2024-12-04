Overview
********

This sample is used to verify GPDMA from ST.
github: https://github.com/lkoudelka/zephyr/tree/wba_cyclic_DMA


Building and Running
********************

Build and flash the sample as follows:

.. code-block:: console

    west build -p -b nucleo_wba55cg -s samples/drivers/uart/uart_wba_dma -t flash

Sample Output
=============

.. code-block:: console

    *** Booting Zephyr OS build v3.7.0-9-gcf7cc0ae6b3c ***


Input characters and console will echo input characters.