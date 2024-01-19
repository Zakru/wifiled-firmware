NeoPixel LED strip controllable over TCP. Written in Rust and running on Embassy,
the async embedded library.

## Example stack

* Firmware running on Raspberry Pi Pico W
* Firmware listens on a TCP port
* Node-RED in LAN translates MQTT messages to TCP packets
* Mosquitto server in LAN
* Any MQTT client for controlling
