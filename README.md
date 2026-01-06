# Linkit V4 Core

Welcome to the Linkit V4 Core repository! This repository contains the firmware for the Linkit V4 device, representing the next generation evolution of the [CLS-Argos-Linkit-CORE](https://github.com/arribada/CLS-Argos-Linkit-CORE) platform.

## Project Evolution

Linkit V4 is an update of the original Linkit platform (previously used with Horizon and Linkit V3 boards with Artic R2 chipset for satellite communication). While the previous generation relied on the Artic R2 chipset, **Linkit V4** introduces significant architectural changes to support new satellite communication modules:

- **KIM module** (CLS) - See [kim-only branch](https://github.com/arribada/linkit-v4-hw/tree/kim-only)
- **SMD module** (Arribada) - See [smd-only branch](https://github.com/arribada/linkit-v4-hw/tree/smd-only)

Hardware repository: [linkit-v4-hw](https://github.com/arribada/linkit-v4-hw)

## Key Architectural Changes

The main differences between Linkit V3 and Linkit V4:

### Communication Interfaces
- **GPS Module**: Upgraded from ML8 to **M10Q** (u-blox), now connected via UART
- **Satellite Module**: Changed from SPI to **UART communication** for both KIM and SMD modules
- **Debug Interface**: UART debug has been reassigned to **USB** to free up hardware UART resources

### Interface Summary
| Component | Interface | Notes |
|-----------|-----------|-------|
| GPS (M10Q) | UART | Previously ML8 on different interface |
| Satellite Module (KIM/SMD) | UART | Previously SPI for Artic R2 |
| Debug | USB | Previously dedicated UART |
| Bluetooth | Existing | No change |

This firmware is compatible with the GenTracker phone app and works seamlessly with the Pylinkit python tool.

**Wiki Navigation**

* [Home](https://github.com/arribada/CLS-Argos-Linkit-CORE/wiki/1-%E2%80%90-Home)
* [Overview](https://github.com/arribada/CLS-Argos-Linkit-CORE/wiki/2-%E2%80%90-Overview)
* [User Guide](https://github.com/arribada/CLS-Argos-Linkit-CORE/wiki/3-%E2%80%90-User-guide)
* [Programmer Guide](https://github.com/arribada/CLS-Argos-Linkit-CORE/wiki/4-%E2%80%90-Programmer-guide)

For detailed instructions and documentation, please refer to the [wiki](https://github.com/arribada/CLS-Argos-Linkit-CORE/wiki).

Feel free to ask questions on the [Linkit Forum.](https://forum.arribada.org/c/linkit-core/8)

# Hardware Compatibility

## Linkit V4
* Linkit V4 with KIM module (CLS)
* Linkit V4 with SMD module (Arribada)

## Legacy Hardware (use [CLS-Argos-Linkit-CORE](https://github.com/arribada/CLS-Argos-Linkit-CORE))
* Horizon Board (Artic R2)
* Linkit V3 CORE (Artic R2)
* Linkit Underwater
* Linkit Surfacebox

# Latest Version

## Linkit V4
* [Linkit V4 Hardware](https://github.com/arribada/linkit-v4-hw):
  - [KIM module branch](https://github.com/arribada/linkit-v4-hw/tree/kim-only)
  - [SMD module branch](https://github.com/arribada/linkit-v4-hw/tree/smd-only)
* Linkit V4 Core Firmware: In development
* [PyLinkit](https://github.com/arribada/CLS-Argos-Linkit-pylinkit): Compatible with V4
* GenTracker: Compatible with V4

## Legacy (Linkit V3)
* [Linkit V3 Core Hardware](https://github.com/arribada/CLS-Argos-Linkit-Hardware): V3.2d
* [Linkit V3 Core Firmware](https://github.com/arribada/CLS-Argos-Linkit-CORE): [V3.4.3](https://github.com/arribada/CLS-Argos-Linkit-CORE/releases/tag/v3.4.3)
* [PyLinkit](https://github.com/arribada/CLS-Argos-Linkit-pylinkit): [V3.4.0](https://github.com/arribada/CLS-Argos-Linkit-pylinkit/releases/tag/v3.4.0)
* GenTracker: Last updated on 29/01/2024


# License

# How to Contribute

Thank you so much for offering to help out. We truly appreciate it.

If you'd like to contribute, start by searching through the issues and pull requests to see whether someone else has raised a similar idea or question.

If you decide to add a feature to this library, please create a PR following these best practices:

* Change as little as possible. Do not sumbit a PR that changes 100 lines of whitespace. Break up into multiple PRs if necessary.
* If you've added a new feature document it with a simple example sketch. This serves both as a test of your PR and as a quick way for users to quickly learn how to use your new feature.
* If you add new functions also add them to keywords.txt so that they are properly highlighted in Arduino. Read more.
* Please submit your PR using the release_candidate branch. That way, we can merge and test your PR quickly without changing the master branch


# Owners

<p align="center">
  <img src="https://www.cls-telemetry.com/wp-content/uploads/2021/06/logo-cls.png" alt="Image Description" width="50%">
</p>
<p align="center">
Collaboration with
</p>
<p align="center">
  <img src="https://arribada.org/wp-content/uploads/2022/01/arribada_web_logo_g.svg" alt="Image Description" width="50%">
</p>
