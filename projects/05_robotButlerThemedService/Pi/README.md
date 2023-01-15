# Raspberry Pi Development

The Raspberry Pi is responsible for:
 - receiving image from the ESP board via (wifi) websocket
 - process the image: face recognition 
 - send back result to the ESP board

Hardware:
 - Currently using a Pi 4 B 2018.
 - Initially, we had a Raspberry Pi Zero W V1.1, 2017, architecture ARMv6Z (32-bit), [see wiki's page](https://en.wikipedia.org/wiki/Raspberry_Pi#Specifications). This version of Pi Zero seems to create some complication, e.g. cannot SSH from VS Code because it is not a supported architecture ([github issue](https://github.com/microsoft/vscode-remote-release/issues/669)).


## Installation

1) Using Raspberry Pi Imager, burn a "Raspberry Pi OS (64-bit)" which includes a Raspberry Pi Desktop.
    1. Download Raspberry Pi Imager
    2. Insert an empty SD card into your PC
    3. Open the Imager software, select an OS (e.g. lite 32bit version), and configure it: enable SSH, set your WiFi, and user and password.
    4. Write
    5. Remove SD card from PC
    6. Insert it into your Raspberry Pi 

2) Clone the git repo (or download the zip file), then move (cd) to the Pi subfolder where this README.md is.

3) Use the [setup.sh](./setup.sh) script to install packages related to cmake and mosquitto, packages listed in requirements.txt, and enable mosquitto.service. Please create your own virtual environment if needed.

- Set write to the setup.sh:
```
chmod +x ./setup.sh
```

- Execute the script
```
./setup.sh
```

- Check installations
```
python3 -V
pip3 -V
```

## Debug steps for facial recognition

[Our initial version of facial recognition](./face-req-v1/) running on a PiZero had very low FPS.
We debugged from scratch and used a local desktop PC (Intel i5-3570K CPU @ 3.40GHz, 16.0 GB RAM, 64-bit Windows 10 Pro, Samsung SSD 860 EVO 1TB, NVIDIA GeForce RTX 2060, webcam USB Logitech C270 HD 720p), see code [in this folder](./face-req-v2/).

Step 1:
 - cap = imutils.video.WebcamVideoStream ([Github's page](https://github.com/PyImageSearch/imutils/blob/master/imutils/video/webcamvideostream.py)), 
 - cap.read()
 - cv2.imshow() with computed fps
 - results: 120 FPS on average on 5 runs

Step 2:
 - use of an optimized model: https://google.github.io/mediapipe/solutions/face_detection.html
 - results: 30 FPS on average with face recognition and facial landmarks (eyes, nose, and mouth)

## References
Raspberry Pi 4 Facial Recognition: Full Tutorial posted - https://www.tomshardware.com/how-to/raspberry-pi-facial-recognition, [RaspberryPi Facial Rec](https://github.com/carolinedunn/facial_recognition/)

WebSocket source: https://nerdhut.de/2019/05/07/websockets-esp8266-raspberry-pi/

### Installation remark on Face Recognition with OpenCV on PiZero

For reference only, we keep details on the attempt to install OpenCV on a PiZero 2017 with ARMv6Z 32-bit architecture.

Before anything, we created a virtual environment, [example of guide](https://realpython.com/python-virtual-environments-a-primer/), to safely install packages.

We attempt an initial build from scratch following a tuto which use a Raspberry 4B with Desktop OS. This was our first attempt following this [source guide](https://core-electronics.com.au/guides/face-identify-raspberry-pi/#Long) and took an entire day. No apparent error occured, but we encounter the error that cv2 module is not found.

We finally switched to install opencv-contrib-python package, precompiled and listed on pypi.org following this [guide](https://singleboardblog.com/install-python-opencv-on-raspberry-pi/) and using [Pypi package page](https://pypi.org/project/opencv-contrib-python/4.5.3.56/). As explained in the guide, we use a opencv-contrib-python, because we need some extra module from their [list](https://docs.opencv.org/4.5.4/) to perform face recognition.

We tried newer version (4.6.0.66 and below) but no precompiled versions were available. As explained in the guide, the sizes were too important: "download only prebuilt binaries around 20-30 MB’s. If pip starts downloading a .tar.gz file around 150 MB and starts building wheels, it means that a precompiled binary is not available for that version".

Launch python and import cv2 confirm the successful installation:
```
python3
import cv2
cv2.__version__
```