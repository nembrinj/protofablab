# Smart Intercom

`Turn an existing intercom into a smarter one.`

When renting a flat or apartment, the pre-installed intercom system is often very basic and does not support any interesting features. Our `smart intercom` is a device that can be placed on top of an existing intercom system and add smart features to it.

The smart intercom device is able to detected the doorbell and will send a push notification to a registered browser application, that can also be used on a mobile phone. The user is then able to trigger the opening of the door in this application.

![](images/schema.png)

## Intercom System - ESP32 Feather

The smart intercom device itself is an `ESP32 Feather` equipped with a sound sensor, a servo motor and a battery. It is connected to a local wifi network to communicate with the server.

The doorbell detection algorithm runs on this device.
If the ringing of the doorbell is detected, a message will be sent over MQTT to the server.
The parameters of the algorithm can be configured to match different doorbells.
The device listens for configuration changes on specific MQTT topics.

The smart intercom also listens for a command to open the door. In this case, the servo motor would be activated to press the button on the underlying intercom.
This does not work in our prototype, since we were not able to integrate a working servo motor with the system.

![](images/device.jpg)

    TODO intercom setup

## Server - Raspberry Pi

A `Raspberry Pi`, located in the same local network as the intercom, acts as the server. It hosts the web application and its database and also the MQTT broker to communicate with the intercom.

The different parts of the server are containerized. They are built and run using `docker-compose`. The following containers exist:

- database
  - A postgres database running on the local port `5432`. It is only beeing used by the api.
- api
  - A nodeJS application providing a REST Api for the web application using [koa.js](https://koajs.com/).
  - Communicates with the intercom over MQTT.
  - Sends push notifications to the web application using [Web Push](https://web.dev/notifications/).
- nginx
  - Serves the pre-build angular web application on ports `80` and `443`.
  - Redirects requests to https://protofablab.ch/api to the api application.
- certbot
  - Is only used initially to generate the SSL certificate for the web application.
- mqtt
  - The MQTT broker running on port `1883`.
  - Used by intercom and api application.

### Installation

To install the smart intercom on the raspberry, first prepare the pi by following these chapters of the [Raspberry Pi Tutorial](../../topics/02_raspPI/README.md):

- Write Image to SD-Card
- Connect to your raspberry pi
- Port forwarding
- DNS setup

After that, follow [these steps](./PI_SETUP.md).

## Using the smart intercom
