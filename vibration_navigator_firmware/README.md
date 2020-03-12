# Vibration Navigator firmware

This package contains firmware sources of vibration navigator.

## Requirements

Following tools are required.

- [platformio](https://pypi.org/project/platformio/)

## How to build and burn the firmware

Please connect M5Stack to your PC usb port.

```
$ roscd vibration_navigator_firmware/firmware
$ rosrun rosserial_arduino make_libraries.py ./lib
$ pio run -t upload --upload-port <M5Stack serial port>
```
