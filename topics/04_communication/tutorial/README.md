# Tutorial 

## What is WebSockets exactly ?

WebSockets is the specification for the usage of the WebSocket protocol, a full-duplex application-layer protocol created in 2008 with the advent of HTML5. It allows easy communication over TCP between two machines, a client and server. The main advantages of this approach lie in primarily two areas :
- WebSockets use the same ports as HTTP/S (a connection using the latter protocol can actually be upgraded to one using the WebSocket protocol using a simple packet containing an "upgrade" header), thus requiring little to no network configuration.
- WebSocket messages can be sent without being requested or sending permission being granted (without polling), unlike HTML, thus considerably reducing the overhead and latency of communication.


## When should you use it ?

It has already been established during this course that MQTT is the protocol of choice for message passing between IoT devices, as it is not very complex and has a low overhead. However, sometimes, we require more.

As an example, an IoT network is not very useful without a client to connect to it to visualise or request information, most often in the form of a web application, but connecting a web browser to a MQTT server is not really practical. Furthermore, the components of some IoT infrasctructures might be very remote from eachother but still need to communicate efficiently, thus requiring a potentially complex infrastructure.

In both of those cases, using the WebSocket protocol's advantages make it a good choice to simplify the architecture of an IoT network. However, since WebSocket is not a very malleable protocol, unlike MQTT whose architecture can be extended and configured to the user's desire, its usage in an IoT context should be limited.

It should also be noted that using the WebSocket protocol as a backbone for communication over MQTT is also possible, and combines the advantages of both to ensure an easy to configure and reliable connection to client machines or remote infra structures.

## Practical part

WebSocket is a well-supported protocol. Several implementations exist for javascript (ref), Arduino (ref) and Python (ref), among others. We will mainly showcase the use of WebSockets in these three contexts during this tutorial with the help of two different examples extracted from the course which we will modify for our purpose.

### Example 1, VEML sensor and Arduino hosting a web server

TODO

### Example 2, adding a Raspberry Pi running a flask server

TODO
