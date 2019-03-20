# ROS driver for Texas Instruments INA219 current/power monitor with I2C interface

This is a ROS node for the INA219. The chip is notably used in the following products:

* [Adafruit INA219 High Side DC Current Sensor Breakout](https://www.adafruit.com/product/904)

There should be no dependencies besides libi2c-dev.

## Parameters:

* **device** (string) -- the path to the i2c device. Default is /dev/i2c-1. Use i2cdetect in the i2c-tools package to find out which bus your IMU is on.
* **address** (int) -- the i2c address of the INA219. Default is 0x40.
* **calibration** (int) -- the value for the INA219 calibration register. Default is 4096.
* **rshunt** (double) -- the shunt resistance in Ohms. Default is 0.1 which is what the Adafruit board uses.
* **publish_voltage_shunt** (bool) -- Publish voltage_shunt topic? Default false.
* **publish_voltage_bus** (bool) -- Publish voltage topic? Default true.
* **publish_power** (bool) -- Publish power? Default false.
* **publish_current** (bool) -- Publish current? Default true.

## Publishers
* **current** (std_msgs/Float32) -- current in amps. Enabled by default.
* **voltage** (std_msgs/Float32) -- the bus voltage (e.g. battery voltage, if you are using this on the high side). Enabled by default.
* **voltage_shunt** (std_msgs/Float32) -- the shunt voltage (voltage across the shunt resistor). Disabled by default.
* **power** (std_msgs/Float32) -- power in watts. Disabled by default.

## Subscribers
None.

## Services
None.

