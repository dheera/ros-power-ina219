# ROS driver for Texas Instruments INA219 current/power monitor with I2C interface

This is a ROS node for the INA219. The chip is notably used in the following products:

* [Adafruit INA219 High Side DC Current Sensor Breakout](https://www.adafruit.com/product/904)

There should be no dependencies besides libi2c-dev.

## Parameters:

* **device** -- the path to the i2c device. Default is /dev/i2c-1. Use i2cdetect in the i2c-tools package to find out which bus your IMU is on.
* **address** -- the i2c address of the INA219. Default is 0x40.

## Publishers
* **current**
* **voltage**
* **power**

## Subscribers
None.

## Services
None.

