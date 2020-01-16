# Firmware Configuration

The most recent versions of ROSflight allow you to specify the hardware setup of your aircraft in greater detail. These settings are dependent on your choice of flight controller. ROSflight does not support this feature on the Naze/Flip32.

For each of a number of devices, there are several choices of configuration. Such configurations may specify the port, the protocol, or other settings for using the device. Many devices can be disabled entirely. Some devices may also have settings via [parameters](parameter-configuration.md).

## Getting Current Configurations
The current configuration for a device can be read through the ROS service `config_get`, which requires `rosflight_io` to be running. The only parameter for the service is the name of the device. The name is case insensitive, and underscores may be used for spaces. For example, all of the following are valid:
```
rosservice call /config_get gnss
rosservice call /config_get "Battery Monitor"
rosservice call /config_get battery_monitor
```
Here is an example of a command and the response
```
$ rosservice call /config_get battery_monitor
successful: True
configuration: "ADC3 on Power"
message: ''
```

##Setting Configurations
Setting configurations is done similarly to getting them, but using the `config_set` service, which also requires `rosflight_io` to be running. The service takes the name of the device and the name of the configuration as parameters.

The service is flexible in the name of the configuration. It is case insensitive and will recognize configurations from most single words in the name. For example the configuration `ADC3 on Power` may be called `adc3` or `power`. The keyword `default` refers to the default configuration for a device. In addition, the number for the configuration may be used. However, due to a quirk in rosservice, this must be prefaced with `!!str` and enclosed in single quotes when calling from the command line. All of the following examples are valid, and the first four are equivalent.

```
rosservice call /config_set gnss "UART1 on Main"
rosservice call /config_set gnss uart1
rosservice call /config_set gnss main
rosservice call /config_set gnss '!!str 1'
rosservice call /config_set airspeed disabled
rosservice call /config_set serial default
```

The service response indicates whether the configuration was successfully set and if a reboot is required. A short message may also be included. When the service fails to set a configuration, it is usually because the firmware found a conflict with another, existing configuration (e.g. putting two things on the same port). The service may also fail if a configuration or device does not exist. An example of the response from setting a configuration is shown below.

```
$ rosservice call /config_set gnss flexi
successful: False
reboot_required: False
message: "Port is used by airspeed sensor."
```
This response indicates that the configuration could not be set because the airspeed sensor uses the same port. It also indicates that no reboot is needed (in this case, because the configuration was not changed).

##Saving Configurations
Configurations are saved by calling the `memory_write` service. This service also saves parameters.
##Configurations
Available devices and configurations are dependent on the flight controller. The Naze/Flip32 does not support changing configurations.

For all boards, the default configuration is configuration #0.
###OpenPilot Revolution (Revo) Configurations
####Serial
See [Using Secondary Serial Links](hardware-setup.md#using-secondary-serial-links) for more details on this setting.

| Configuration | Number | port | Notes |
| ------------- | ------ | ---- | ----- |
| VCP over USB  | 0 |Micro USB|
| UART1 on Main|1| Main|
| UART2 on Flex-IO|2| Flex-IO|
| UART3 on Flexi|3| Flexi|

####RC
| Configuration | Number | port | Notes |
| ------------- | ------ | ---- | ----- |
| PPM on Flex-IO|0|Flex-IO|Does not conflict with UART on the Flex-IO port|
|SBUS on Main|1|Main||

####Airspeed
| Configuration | Number | port | Notes |
| ------------- | ------ | ---- | ----- |
|Disabled|0|None||
|I2C2 on Flexi|1|Flexi|Multiple I2C devices can share the port.|

####GNSS

| Configuration | Number | port | Notes |
| ------------- | ------ | ---- | ----- |
|Disabled|0|None||
|UART1 on Main|1|Main||
|UART2 on Flexi-IO|2|Flex-IO||
|UART3 on Flexi|3|Flexi||

####Sonar
| Configuration | Number | port | Notes |
| ------------- | ------ | ---- | ----- |
|Disabled|0|None||
|I2C2 on Flexi|1|Flexi|Multiple I2C devices can share the port.|

####Battery Montior
| Configuration | Number | port | Notes |
| ------------- | ------ | ---- | ----- |
|Disabled|0|None||
|ADC3 on Power|1|PWR / Sonar||

####Barometer

| Configuration | Number | port | Notes |
| ------------- | ------ | ---- | ----- |
|Disabled|0|None||
|Onboard Barometer|1|None||

####Magnetometer

| Configuration | Number | port | Notes |
| ------------- | ------ | ---- | ----- |
|Disabled|0|None||
|Onboard Magnetometer|1|None||
