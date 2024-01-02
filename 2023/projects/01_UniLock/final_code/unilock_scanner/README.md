
To set up and start the scanner software on your aiy vision pi do the following.

0. (optional) Flash the custom scanner image onto your rasberry pi zero :

   If you got your hands on a AYI vision kit you can just the reset of this guide. since you can just flash the custome image files using  the rasberry pi imaging tool. 

   Just get the imager, and download custom image from the realase page. https://www.raspberrypi.com/software/


1. First, clone the github repo : 

```bash

   git clone https://github.com/jack-unibe/FaceGuard.git

```

2. Install the python dependencies :

   ```bash
   pip install -r requirements.txt
   ```

4. Navigate to qr scanner folder :

   ```bash
   cd ./unilock_scanner
   ```

5. Open up port 5000 on your aiy vision pi : 

   ```bash
   ufw allow 5000
   ```

6. Start the server : 

   ```bash
   bash ./start.sh
   ```

From here on you can just navigate to http://<your-rip-ip/hostname>:5000 to activate the cammera and scan qrcodes 
