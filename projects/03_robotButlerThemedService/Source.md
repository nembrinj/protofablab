# Sources Project

## Arduino 
### Memory issues due to communication
- https://www.forward.com.au/pfod/ArduinoProgramming/ArduinoStrings/index.html#reboot
- https://forum.arduino.cc/t/using-reserve-efficiently/917587

### Serial communication
 - https://docs.arduino.cc/learn/built-in-libraries/software-serial

### Coroutine - multitask - mutlithread
 - https://forum.arduino.cc/t/coroutines-library/247257
- https://roboticsbackend.com/how-to-do-multitasking-with-arduino/
 - https://forum.arduino.cc/t/how-to-run-multiple-functions-independently/93062/3
 - https://www.instructables.com/Parallelism-and-Much-More-on-the-T-Pico-C3/
 - https://www.instructables.com/Parallelism-and-Much-More-on-the-T-Pico-C3/
 - https://www.arduino.cc/reference/en/language/functions/time/millis/
 - https://stackoverflow.com/questions/67734115/how-to-use-multithreading-with-websockets 
 - https://stackoverflow.com/questions/3221655/python-threading-string-arguments 

### Inspiration for thread
 -https://roboticsbackend.com/arduino-protothreads-tutorial/

### Servo
- https://forum.arduino.cc/t/arduino-sketch-to-pan-servo-left-and-right/485401
 - https://www.arduino.cc/reference/en/libraries/servo/
 - https://docs.arduino.cc/learn/electronics/servo-motors
 - https://forum.arduino.cc/t/how-do-you-get-servo-position-in-arduino/4760/5

## Pi
### Wifi setup, headless
 - https://howchoo.com/g/ndy1zte2yjn/how-to-set-up-wifi-on-your-raspberry-pi-without-ethernet 
 - https://www.raspberrypi-spy.co.uk/2017/04/manually-setting-up-pi-wifi-using-wpa_supplicant-conf/

 ### PiZero SSH
  - https://forums.raspberrypi.com/viewtopic.php?t=267267
  - https://forums.raspberrypi.com/viewtopic.php?t=296434

### Ping remote 
 - https://raspberrypi.stackexchange.com/questions/105080/how-to-remote-to-raspberry-pi-from-outside-local-network 

### OpenCV 
- https://core-electronics.com.au/guides/face-identify-raspberry-pi/ 
- https://circuitdigest.com/microcontroller-projects/raspberry-pi-and-opencv-based-face-recognition-system 
 - https://pythonexamples.org/python-opencv-read-image-cv2-imread/ 
 - https://indianaiproduction.com/show-image-cv2-imshow/
 - https://github.com/PyImageSearch/imutils/blob/master/imutils/video/videostream.py 

### Optimizing
 - https://pyimagesearch.com/2017/10/09/optimizing-opencv-on-the-raspberry-pi/
 - https://pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/#:~:text=The%20%E2%80%9Csecret%E2%80%9D%20to%20obtaining%20higher,method%20is%20a%20blocking%20operation 
 - https://stackoverflow.com/questions/52068277/change-frame-rate-in-opencv-3-4-2

### Face recognition
 - https://face-recognition.readthedocs.io/en/latest/face_recognition.html
  - https://face-recognition.readthedocs.io/en/latest/_modules/face_recognition/api.html 

#### Depth calculus
 - https://www.youtube.com/watch?v=jsoe1M2AjFk

#### Face landmarks
 - https://datagen.tech/guides/face-recognition/facial-landmarks/


## ESP

### Memory
- https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html
- https://forum.arduino.cc/t/esp32-cam-frame-buffer-manipulation-possible/913051
 - https://forum.arduino.cc/t/esp32-cam-frame-buffer-manipulation-possible/913051
 - https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/external-ram.html 

### Face recognition 
 - https://github.com/espressif/esp-who 

### Camera on webpage
 - https://www.youtube.com/watch?v=gE3DyYze8c8&ab_channel=ThatProject 
 - https://www.mischianti.org/2022/07/26/esp32-cam-control-camerawebserver-from-your-own-web-page-3/ 
 - https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer

### Issue frame distorted
 - https://github.com/espressif/esp32-camera/issues/172

### Camera settings
 - https://randomnerdtutorials.com/esp32-cam-ov2640-camera-settings/

### Issue performance
 - https://github.com/espressif/arduino-esp32/issues/4655
 - https://github.com/espressif/esp32-camera/issues/15
 - https://www.reddit.com/r/esp32/comments/vndbh7/how_to_increase_fps_on_esp32cam/


## Communication
### JSON
 - https://arduinojson.org/v6/troubleshooter/#runtime/deserialization/incompleteinput/string  
 - https://arduinojson.org/v6/doc/deserialization/
 - https://arduinojson.org/v6/api/dynamicjsondocument/
 - https://arduinojson.org/v6/api/staticjsondocument/
 - https://arduinojson.org/v6/api/staticjsondocument/
 - https://forum.arduino.cc/t/multiple-json-data-parasing/450496
 - https://forum.arduino.cc/t/how-to-deserealize-json-with-nested-json/912783
 - https://www.youtube.com/watch?v=NYP_CxdYzLo&ab_channel=BrianLough

### ESP32 - Arduino

#### UART
 - https://microcontrollerslab.com/esp32-uart-communication-pins-example/
 - https://www.youtube.com/watch?v=YVPumD16Y_Y&t=321s
 - https://forum.arduino.cc/t/sending-data-from-esp32-to-arduino-uno/695953

##### Inspiration code
- https://www.programmingboss.com/2021/04/esp32-arduino-serial-communication-with-code.html

##### ESPSoftwareSerial
 - https://www.arduino.cc/reference/en/libraries/espsoftwareserial/

##### Software serial issue
- https://forum.arduino.cc/t/softwareserial-not-working/432349

### Pi - ESP32
#### UDP 
 - https://www.aranacorp.com/fr/communication-udp-entre-raspberry-pi-et-esp32/amp/ 

#### HTTP POST (half duplex communication)
 - https://randomnerdtutorials.com/esp32-esp8266-raspberry-pi-lamp-server/

#### Websocket (full duplex communication)
 - https://docs.aiohttp.org/en/stable/_modules/aiohttp/web_ws.html 
 - https://docs.aiohttp.org/en/v3.7.4/web_advanced.html 
 - https://docs.aiohttp.org/en/stable/web_reference.html

#####  Tuto inspiration
 - https://techtutorialsx.com/2021/12/06/esp32-websocket-server-broadcast-messages/
- https://github.com/Links2004/arduinoWebSockets/blob/master/examples/esp32/WebSocketClient/WebSocketClient.ino

##### Issue websocket blocks handler indefinitely
 - https://github.com/aio-libs/aiohttp/issues/4877  
 - https://forum.arduino.cc/t/websocket-server-loop-behaviour/1002500

###### Fast websocket transfer
 - https://forum.espruino.com/conversations/302083/
 - https://stackoverflow.com/questions/7730260/binary-vs-string-transfer-over-a-stream 

#### Stream video from ESP32 to Pi
 - https://gpiocc.github.io/learn/raspberrypi/esp/ml/2020/11/08/martin-ku-stream-video-from-esp32-to-raspberry-pi.html 

### MQTT
 - http://www.steves-internet-guide.com/subscribing-topics-mqtt-client/
 - https://help.ivanti.com/wl/help/en_US/VScript/1.2/MQTTClient_subscribe.htm
 - http://www.steves-internet-guide.com/subscribing-topics-mqtt-client/

#### Issue address already in use
- https://community.openhab.org/t/solved-mqtt-broker-not-working-probably-error-address-already-in-use/53366
 - https://forums.raspberrypi.com/viewtopic.php?t=316291

## Assets
### Theme “One Piece”
#### Boa Hancock
 - https://www.reddit.com/r/BoaHancock/comments/vrgj6l/beautiful_face/
 - https://www.pinterest.ch/pin/350788258482390358/?mt=login
  - https://encrypted-tbn3.gstatic.com/images?q=tbn:ANd9GcQrMQ7nrlpAjjLKOp07jQ9j3807MiFmULm-OkmRTU1dL4t2Wt3f 
  - https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcQLOrDsIdlf4YBzqTSH5_alYLZ5XouC6FX8_V2Ej7_-Y-o95FVB 

#### Roronoa Zoro
 - https://e7.pngegg.com/pngimages/555/12/png-clipart-roronoa-zoro-monkey-d-luffy-one-piece-vinsmoke-sanji-one-piece-mammal-face.png 
 - https://w7.pngwing.com/pngs/824/19/png-transparent-roronoa-zoro-kazuya-nakai-male-seiyu-zoro-miscellaneous-face-head.png
 - https://i.pinimg.com/564x/2a/bf/ef/2abfef7bc4874e03158f4cab8ce26df3.jpg 
 - https://www.pinterest.ch/pin/625648573207110682/


