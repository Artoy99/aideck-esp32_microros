# AI-Deck ESP32 with uROS firmware
This repository add a publisher for uROS inside the ESP32 firmware of the crazyflie's AI-Deck. The target listener is the PC with an agent running on. It is a demo that aim to be improve.

**NOTE:** If you want the base ESP32 firmware for AI-Deck, visite [Esp32 AI-Deck repository].

We are using these configurations to make the crazyflie and AI-Deck work:
- Default Crazyflie NRF51 firmware (version 2022.0, flashed with CfClient)
- Default Crazyflie STM32 firmware (release 2022.05, flashed with USB)
- Default AI-Deck GAP8 bootloader firmware (release 2022.05, flashed with JTAG)

[Esp32 AI-Deck repository]: https://github.com/bitcraze/aideck-esp-firmware

## Prerequisites
After cloning the repository, do not forget to update the submodules.
```
git submodule init
git submodule update
```

In order to build the program, you will need to install the [Esp32 IDF]. This project used the 4.3.1 version of it.

Openocd is needed to flash the board with JTAG. Bitcraze provide an docker to do so but this has not been tested. 

[Esp32 IDF]: https://github.com/espressif/esp-idf

## Compile
### IDF
To build, from the root of the repository:
```
source ~/path/to/idf/export.sh
idf.py build
```

### Docker
**NOTE:** This method has not been tested.

To build the firmware in Docker, use:
```
docker run --rm -it -v $PWD:/module/ --device /dev/ttyUSB0 --privileged -P bitcraze/builder /bin/bash -c "make"
```

## Flash (JTAG)
### Native
To flash the ESP using a JTAG, use:
```
openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f board/esp-wroom-32.cfg -c 'adapter_khz 20000' -c 'program_esp build/bootloader/bootloader.bin 0x1000 verify' -c 'program_esp build/aideck_esp.bin 0x10000 verify reset exit'
```

### Flash using the builder image
If you do not/can not install openocd on your computer, you can use the builder image given by Bitcraze:
```
docker run --rm -it -v $PWD:/module/ --device /dev/ttyUSB0 --privileged -P bitcraze/builder /bin/bash -c "openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f board/esp-wroom-32.cfg -c 'adapter_khz 20000' -c 'program_esp build/bootloader/bootloader.bin 0x1000 verify' -c 'program_esp build/aideck_esp.bin 0x10000 verify reset exit'"
```

## Flash (Over the air)
**NOTE:** This method has not been tested.

A binary can be flashed to the ESP via a Crazyradio using the cfloader

Note: replace the radio address with the appropriate value for your Crazyflie
```
cfloader flash build/aideck_esp.bin deck-bcAI:esp-fw -w radio://0/30/2M
```
