# Tutorial 

## What is WebSockets exactly ?

WebSockets is the specification for the usage of the WebSocket protocol, a full-duplex application-layer protocol created in 2008 with the advent of HTML5. It allows easy communication over TCP between two machines, a client and server. The main advantages of this approach lie in primarily two areas :
- WebSockets use the same ports as HTTP/S (a connection using the latter protocol can actually be upgraded to one using the WebSocket protocol using a simple packet containing an "upgrade" header), thus requiring little to no network configuration.
- WebSocket messages can be sent without prior agreement from the receiver (without polling), unlike HTML. This considerably reduces the overhead and latency of communication.
- Finally, one important point of WebSockets is that it gives web apps an ability to access TCP sockets, which can be very useful for a number of applications.


## When should you use it ?

It has already been established during this course that MQTT is the protocol of choice for message passing between IoT devices, as it is not very complex and has a low overhead. However, sometimes, we require more.

As an example, an IoT network is not very useful without a client to connect to it to visualise or request information, most often in the form of a web application, but connecting a web browser to a MQTT server is not really practical. Furthermore, the components of some IoT infrasctructures might be very remote from eachother but still need to communicate efficiently, thus requiring a potentially complex infrastructure.

In both of those cases, using the WebSocket protocol's advantages make it a good choice to simplify the architecture of an IoT network. However, since WebSocket is not a very malleable protocol, unlike MQTT whose architecture can be extended and configured to the user's desire, its usage in an IoT context should be limited.

It should also be noted that using the WebSocket protocol as a backbone for communication over MQTT is also possible. Such an option is commonly called "MQTT over WebSocket", and is in fact the most convenient way to communicate with an MQTT broker from a web app. It combines the advantages of both to ensure an easy to configure and reliable connection to client machines or remote infrastructures.

## Practical introduction

WebSocket is a well-supported protocol. Several implementations exist for javascript (ref) and Python (ref), among others. We will mainly showcase the use of WebSockets in these two contexts during this tutorial with the help of an example derived from one of the course's exercises.

### Mini-project structure

The project we will use to showcase how the WebSocket protocol can be applied to an IoT consists of four parts :
- an arduino board wired to a VEML7700 lux sensor
- a [remote MQTT broker service](http://maqiatto.com), for convenience, though you can certainly use your own MQTT broker
- a Raspberry Pi zero W, installed with the OS of your choice, we recommend [DietPi](https://dietpi.com), but the image provided during the course works also well.
- a device with a browser to access the web application, and another to host a WiFi hotspot with access to the wider Internet, not just a local hotspot)

*Note that this is an approach into the use of web sockets in an IoT context, web sockets have a lot of options that can be configured (e.g. if you want TLS encryption), and you should refer yourself to the documentation of the library you are using.*

The project is setup so as to compare the different methods of sending sensor data to a web app. As such, the data follows two main paths :
- the sensor data is directly relayed to the web app via the MQTT broker
- the sensor data is relayed to the raspberry pi, which then :
  - sends it back to the MQTT broker on another channel (this allows us to see what kind of delay such an operation would impose
  - sends it directly to the web app via a Web Socket

![Schema of all MQTT and WS connections in the mini project](https://github.com/nembrinj/protofablab/blob/main/topics/04_communication/tutorial/images/Project-connections.png)

### Running the project

Before we started showcasing the IoT uses of WebSockets, we must first configure the project. To do this, setup your mqtt browser and your wifi hotspot, then copy the required informations in each of the files. Finally, copy the files in the `example-project` folder on your devices :
- the files in the `wifi_mqtt_veml7700_raspi` folder must be compiled and copied onto the arduino
- all the other files must go into a single folder on your raspberry pi. You should then execute in that same folder the following command : `pip install -m requirements.txt`. This will install the required libraries to run the project.

### How does it work ?

#### Javascript - WebSockets

Let us now see how opening a web socket works in javascript. This method is used in file `websocket.html` in the `templates` folder.

First, we need to use a library. There are several possibilities, among which is the popular [Socket.io](https://socket.io/), with the ability to run without any framework. It is available via multiple CDNs (we used [Cloudflare's](https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.5.3/socket.io.min.js)), and there are a lot of installation options in the [documentation](https://socket.io/docs/v4/client-installation/).

To start, the socket.io script is first loaded via a CDN.
```
<script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js" integrity="sha512-q/dWJ3kcmjBLU4Qc47E4A9kTB4m3wuTY7vkFJDTZKjTs8jhyGQnaUrxa0Ytd0ssMZhbNua9hE+E7Qv1j+DyZwA==" crossorigin="anonymous"></script>
```
Then, the implementation is very simple. First create a web socket :
```
var socket = io();
```
Finally, the interaction with the server, to which we were automatically connected, as it is the same server that provided us with the page the web app runs on, is handled by the function `on()` :

```
socket.on('connect', function() {
    // confirm connection with socket.emit(), which is the function for sending messages to the server, for example :
    socket.emit('my event', {data: 'I\'m connected!'});
});
socket.on("message",function(message){
     // based on the content of the message, modify the content of your webpage, in our case :
     document.getElementById("luxValue").innerHTML = message
});
```

Websockets in javascript are therefore very intuitive to use, and a great tool for live element updates.

#### Javascript - MQTT over WebSockets

Accessing an MQTT broker from a browser using only MQTT is very difficult. Since MQTT works on a raw TCP connection, this would require giving access to TCP sockets to any web page, which is from a security standpoint not very advisable.

Thankfully, there are several options to easily access an MQTT browser over WebSockets, a method that is easily supported in most brokers (see [here](https://mosquitto.org/man/mosquitto-conf-5.html) for Mosquitto). One of the more common libraries to use is [paho](https://www.eclipse.org/paho/index.php?page=clients/js/index.php), which we used as a [CDN-delivered script](https://cdnjs.cloudflare.com/ajax/libs/paho-mqtt/1.0.1/mqttws31.js) in files `mqtt.html` and `directMqtt.html`.

Using paho is very similar to socket.io, with small differences. We first start by loading the paho library :
```
<script src="https://cdnjs.cloudflare.com/ajax/libs/paho-mqtt/1.0.1/mqttws31.js" type="text/javascript"></script>
```

We then create a new mqtt client, along with some callbacks, and connect it to the broker :

```
// Create a client instance
client = new Paho.MQTT.Client("maqiatto.com", 8883, "clientId"); // change if your broker is different

// set callback handlers
client.onConnectionLost = onConnectionLost;
client.onMessageArrived = onMessageArrived;
// connect the client
client.connect({
  onSuccess: onConnect,
  // change both following values
  userName : "<username on maqiatto.com>",
  password : "<your maqiatto.com account's password>",
});
```

We then need functions to subscribe to topics. This is done when a connection has been established, with code similar to the following :

```
// called when the client connects
function onConnect() {
  // Once a connection has been made, make a subscription and send a message.
  client.subscribe("<username on maqiatto.com>/protofablab/lux");
}
```

Finally, when a message arrives, we need to process it. This can be done by using the following function :

```
// called when a message arrives
function onMessageArrived(message) {
  // modify your document/execute code accordingly, for example, in our case :
  document.getElementById("luxValue").innerHTML = message.payloadString;
}
```

Finally, there are other functions that can be implemented to handle particular cases, for example lost connections, for which we can use the `onConnectionLost()` function, allowing us to gracefully handle errors.

#### Python + Flask - WebSocket server

We have seen several client implementations, but those clients must receive their messages from somewhere, particularly the pure websockets clients. This is where socket.io comes into play again.

Socket.io is available in python via an extension to the Flask library. This is very helpful, since the network configuration is already done, and we only need to start sending messages.

To start, we load the socket.io library like follows, and we create a web socket
```
from flask_socketio import SocketIO
app = Flask(__name__) // start a Flask server
socketio = SocketIO(app) // create a socket on that server
``` 

Then, we only need to define our function to send messages, this is in our ase done by using the `SocketIO.send()` (you can also use `SocketIO.emit()`) function in a wrapper function that is called upon reception of an MQTT message. The message that is sent is first formatted as a json string.

```
@socketio.on('message') // handle events of type "message", not needed here
def handle_message(val):
    toSend = dict(
        timestamp = (time.time_ns()) // 1000000,
        message = val
    )
    toSend = json.dumps(toSend)
    socketio.send(toSend,broadcast=True)
    mqtt_client.publish('<your username on maqiatto.com>/flask/lux', toSend)
```





