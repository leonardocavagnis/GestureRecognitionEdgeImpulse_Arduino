# Gesture Recognition EdgeImpulse Arduino 

## Description

This repository contains an Arduino sketch for gesture recognition using Edge Impulse.

For a detailed description and explanation, please refer to the external article [here](https://medium.com/@leonardocavagnis/gesture-recognition-with-edge-impulse-and-arduino-0da09c0873d5).

## Hardware Requirements

- [Arduino Giga R1 WiFi board](https://store.arduino.cc/products/giga-r1-wifi)
- [Arduino Giga Display Shield](https://store.arduino.cc/products/giga-display-shield)

## Dependencies

This sketch relies on the following libraries:

- [Arduino_BMI270_BMM150](https://www.arduino.cc/reference/en/libraries/arduino_bmi270_bmm150/) for IMU sensor support
- [Arduino_H7_Video](https://github.com/arduino/ArduinoCore-mbed/tree/main/libraries/Arduino_H7_Video) for display management
- [Arduino_GigaDisplayTouch](https://www.arduino.cc/reference/en/libraries/arduino_gigadisplaytouch/) for touch detection
- [LVGL](https://github.com/lvgl/lvgl) for graphics rendering

Please install these libraries through the Arduino Library Manager before uploading the sketch.

## Authors

This project was developed by [Leonardo Cavagnis](https://github.com/leonardocavagnis).
