## Project: Thingy91-ultrasonic-aws
This project contains sample code to integrate AJ-SR04M ultrasonic sensor with Nordic Semiconductor's Thingy91 and send periodic distance measurements to AWS IoT MQTT server. This work is based on Nordic Semiconductor's [nRF9160:Cloud Client](http://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.4.0/nrf/samples/nrf9160/cloud_client/README.html) sample which was included in [nRF Connect SDK v1.4.0](http://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.4.0/nrf/introduction.html).

## Dependencies
nRF Connect SDK v1.4.0 is associated with the following repositories:
- Zephyr RTOS v2.4.0
- nrfxlib v1.4.0
- MCUboot v1.6.99

## Configuration and settings
Important configuration and settings of the project are stored in "prj.conf" and "Kconfig" files.

## Setting and running
For setting up the development environment and running code on an nRF-9160 development board follow the instructions [here](https://devzone.nordicsemi.com/nordic/nrf-connect-sdk-guides/b/getting-started/posts/nrf-connect-sdk-tutorial) on Nordic Devzone blog.

## Drivers and Libraries used
### Ultrasonic Sensor: AJ-SR04M
Communication of nRF-9160 with ultrasonic sensor is handled by the following peripherals:
- [nrfxlib:GPIO](https://infocenter.nordicsemi.com/topic/drivers_nrfx_v2.4.0/group__nrf__gpio.html)
- [nrfxlib:GPIOTE](https://infocenter.nordicsemi.com/topic/drivers_nrfx_v2.4.0/group__nrf__gpiote.html)
- [nrfxlib:TIMER](https://infocenter.nordicsemi.com/topic/drivers_nrfx_v2.4.0/group__nrf__timer.html)

### AWS IoT communication
Communication of nRF-9160 with AWS IoT MQTT server is handled by the following libraries:
- [Cloud API](http://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.4.0/nrf/include/net/cloud.html#cloud-api-readme)

### LTE and PSM settings
nRF-9160 LTE connection and PSM mode is requested with the help of [LTE link controller](http://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.4.0/nrf/include/modem/lte_lc.html).

In case of questions and queries about this project contact me on my email: [harisahmed94@gmail.com](mailto:harisahmed94@gmail.com).