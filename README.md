# ESP32 CSI-Based TRNG (Experimental / Educational)

> [!WARNING]  
> This project is strictly for learning about hardware random number generators and exploring entropy in wireless signals.
> **Do not use this for cryptographic or security-critical purposes.**

## Overview

This project demonstrates a toy true random number generator (TRNG) using Channel State Information (CSI) from a Wi-Fi ESP32 module. The idea is to explore hardware and environmental sources of entropy and understand how to measure, extract, and test random bits.

It is **not production-grade**: the throughput is low, statistical properties are imperfect, and the random stream has not been fully validated against exhaustive cryptographic tests. This project is intended purely as an *educational exercise* in embedded systems, signal processing, and randomness testing.

## How It Works

1. Capture CSI from the ESP32 Wi-Fi receiver
2. Select usable subcarriers (excluding pilots and DC) and optionally pick evenly spaced carriers to reduce correlation
3. Extract LSB bits from the amplitude of each subcarrier
4. Distill bits across subcarriers using an HMAC extractor
5. Output raw bits over UART to a host computer.


## Caveats & Limitations

1. Low throughput: Typically only a few bytes per second due to limited number of subcarriers and conservative extraction.
2. Known statistical imperfections: Serial correlation coefficients are non-zero; raw bits are not IID.
3. The project does not cover exhaustic cryptographic test suites for TRNGs (Refer [dieharder suite](https://webhome.phy.duke.edu/~rgb/General/dieharder.php)). However, seeding PRNGs like mt19937 with the CSI TRNG passes the suite. 

This means the raw bitstream cannot be used for real crypto without strong post-processing and careful evaluation.

## Hardware & Software Requirements

1. The driver program is a fork of `csi_recv_router` from [espressif/esp-csi](https://github.com/espressif/esp-csi/tree/master/examples/get-started/csi_recv_router) and should work across the supported ESP32 chipsets. The functions in `trng_csi.h` however were only tested on an ESP32C6 devkit.
2. ESP-IDF >= v5.0

## Setup & Usage

1. Build and flash the firmware to your ESP32
2. The firmware will output CSI-derived bits over serial (/dev/ttyACM0 or similar)
3. Bits can be read from the host as a continuous stream via serial

Example using miniterm

```bash
python -m serial.tools.miniterm /dev/ttyACM0 921600
```
