# core/ble_services/

Platform-independent BLE GATT service interface (`ble_service.hpp`).

The concrete BLE stack implementation (Nordic SoftDevice S140) lives in [`../../ports/nrf52840/core/interface/ble_interface.cpp`](../../ports/nrf52840/core/interface/ble_interface.cpp).

DTE traffic over BLE flows through this interface — see [`../protocol/`](../protocol/) for the protocol layer on top.
