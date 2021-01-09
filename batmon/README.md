Jetson Nano battery monitor.

Simple service that monitors and shutdowns the system
when the battery is below some value

This service require the ADS1115 is connected to the
Nano and is handled by the ads1015 linux driver

The battery voltage value is available on user space as the content of

```sh
cat /sys/class/hwmon/hwmon2/device/in3_input
```
