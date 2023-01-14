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

The source code for the smart intercom device is located in the `arduino` subfolder.

Note that since we couldn't figure out how to make the servo motor work, there is no code that handles the pushing of the button, because we were not able to test it. Thus, the source code is mostly about **detecting the doorbell** and **handling MQTT messages**.

### Doorbell detection

We decided to detect the doorbell simply by detecting loud noises. This means we are not trying to recognize the waveform of the doorbell, so we must be carefull not to detect random sounds, such as the door closing.

We faced multiple challenges when writing the detection algorithm. First, the input waveform has a variable DC offset, depending on the gain. To overcome this, we compute the average amplitude over time, and we substract this value from the amplitude. This makes the waveform centered around zero.

[ INSERT SKETCH HERE ]

Another challenge: the sound is a waveform, but we can only read the current amplitude on each iteration. Because the input oscillates, we can't tell whether we are hearing a loud noise during a single iteration. Thus, we have to write an algorithm to detect the "maximum amplitude" over a short period of time.

[ INSERT SKETCH HERE ]

TODO: describes parameters

### MQTT messages handling

TODO

### Setting up the device

To setup the smart intercom device, start by connecting the microphone module to the ESP32 Feather. In our case, we used the Adafruit MAX9814 module with the gain set to `Vdd` (40 dB), but other microphones and different gains should work just as well. The analog output must then be connected to the **A1** pin of the Arduino board.

Unfortunately, we were not able to connect and use the servo motor succesfully, so we won't describe this step here.

In the `arduino` folder, open the `arduino.ino` file with Arduino IDE. Set the string constants at the beginning of the source file to your Wi-Fi and MQTT credentials (the ones that start with `WIFI_` and `MQTT_`). Then plug the ESP32 Feather to your computer and upload the code.

Once done, uplug the board from the computer and plug it to a portable battery. Set it up in such a way that the microphone is as close to doorbell speaker as possible. The device is now ready.

## Server - Raspberry Pi

A `Raspberry Pi`, located in the same local network as the intercom, acts as the server. It hosts the web application and its database and also the MQTT broker to communicate with the intercom.

The different parts of the server are containerized. They are built and run using `docker-compose`. The following containers exist:

- database
  - A postgres database running on the local port `5432`. It is only being used by the api application.
- api
  - A nodeJS application providing a REST API for the web application using [koa.js](https://koajs.com/).
  - Communicates with the intercom over MQTT.
  - Sends push notifications to the web application using [Web Push](https://web.dev/notifications/).
  - Runs on local port `3000`.
- nginx
  - Serves the pre-built angular web application on ports `80` and `443`.
  - Redirects requests to https://protofablab.ch/api to the api application.
- certbot
  - Is only used initially to generate the SSL certificate for the web application.
- mqtt
  - The MQTT broker running on port `1883`, available on the local network.
  - Used by intercom and api application.

### Installation

To install the smart intercom on the raspberry, first prepare the pi by following these chapters of the [Raspberry Pi Tutorial](../../topics/02_raspPI/README.md):

- Write Image to SD-Card
- Connect to your raspberry pi
- Port forwarding
- DNS setup

After that, follow [these steps](./PI_SETUP.md).

## Using the smart intercom

After setting up the smart intercom device and the Raspberry Pi, the web application can be accessed from your phone at https://protofablab.ch (or your chosen domain).

It was only tested using Chrome, but other browsers might also work. The most restricting factor for the choice of browser, is [support of the Web Push API](https://caniuse.com/push-api).

First, the permission to display notifications has to be granted.
A pop-up to allow this should appear automatically when first visiting the site. If it does not, it might be necessary to reset the granted permissions for the website.

<img src="images/screenshot_notification.png" height="500"/>

After giving the necessary permission to the web application. Your phone is now ready to receive a push notification, when the doorbell detection is triggered. This will look like this:

<img src="images/notification.gif" height="500"/>

On the main screen of the application it is now possible to trigger the door opening mechanis by pressing the `Open door` button.

It is also possible to configure the doorbell detection algorithm directly from the web application. The configuration menu can be accessed by pressing the `Configure` button.

<img src="images/screenshot_configuration.png" height="500">

## Limitations

There are some current limitations, that restrict the utility of the smart intercom.

### The servo motor isn't working

We were unable to get a working servo motor to push the door button. This might have been a problem with the `ESP32` or the motors themselves.
Without this functionality the smart intercom is only able to notify us, when someone is at the door and we aren't home.

### Doorbell sound detection is not very accurate

Depending on the sound pattern of the doorbell, it might not be possible to configure the sound detection algorithm in satisfiable way.
Loud noices near the smart intercom device might lead to false positives.

### No authentication

Right now, the web application features no user authentication. This means, anybody could receive notifications when our door bell rings and, far worse, would be able to open our front door, if a working servo motor would be installed.
