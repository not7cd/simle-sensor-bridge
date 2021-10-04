# simle-sensor-bridge
Uses seeeduino XIAO to collect data from sensors and CAN bus and sends it over i2c.

It acts as a bridge between UAVCAN and Pixhawk, since px4 autopilot doesn't support UAVCAN 1.0. 
In our tests it proved unstable and prone to crashes. Sensor bridge should be seen as temporary solution.

In the future this device would only send its temperature readings over UAVCAN.
