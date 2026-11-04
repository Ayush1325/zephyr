.. _snippet-ti-cc13x2x7-ble:

TI CC13x2x7/CC26x2x7 Bluetooth LE Controller (ti-cc13x2x7-ble)
##############################################################

You can build with this snippet by following the instructions in :ref:`the snippets usage page<using-snippets>`.
When building with ``west``, you can do:

.. code-block:: console

   west blobs fetch hal_ti
   west build -S ti-cc13x2x7-ble [...]

Overview
********

This selects the TI BLE5-Stack as Bluetooth LE controller. The stack runs on the same SoC as the
Zephyr Bluetooth host and is linked from a prebuilt library, fetched as a blob of the ``hal_ti``
module.

The stack uses parts of the Bluetooth LE ROM of the SoC, which keeps its data in the first 4 KiB
of SRAM. This snippet removes that region from the SRAM available to Zephyr.

Requirements
************

- A board with a TI CC13x2x7 or CC26x2x7 SoC
- :kconfig:option:`CONFIG_BT`

Limitations
***********

The controller only uses a static random address for scanning and initiating, and generates
resolvable private addresses itself from its resolving list. With
:kconfig:option:`CONFIG_BT_PRIVACY`, the host asks for a resolvable private address or its own
random address (own address type 0x03) instead, which the controller rejects. Active scanning and
connecting as central therefore fail when privacy is enabled. Advertising and the peripheral role
work with privacy.
