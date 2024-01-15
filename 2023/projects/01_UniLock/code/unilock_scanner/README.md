### Scanner

There are two options for setting up the scanner. If you already have a AIY vision kit you can just directly flash our custom image `2023/projects/01_UniLock/code/unilock_scanner/images/unilock-scanner-v1.img.gz`. If not then you can ssh into your Rasberry Pi and setup the software from scratch.  

#### A) Flash the custom image

Assembly the AIY Vision kit and use the [Rasberry Pi imaging tool](https://www.raspberrypi.com/software/) to flash our preconfigured os image `unilock-scanner-v1.img.gz` onto the AIY's sd card. This operation automatically does the initial network and account configuration. From there you can just plug in the Pi and the qr scanner software will activate on boot.

#### B) Install and configure from scratch
    
  Clone the protofab github repo:
  ```bash
   git clone https://github.com/nembrinj/protofablab.git
  ```
  
  Navigate to the `unilock_scanner` directory:
  ```
  cd 2023/projects/01_UniLock/code/unilock_scanner
  ```
  Install the python dependencies:
  ```bash
  pip install -r requirements.txt
  ```
  
  Open up port 5000 on rasberry PI:
  ```bash
  ufw allow 5000
  ```

  Modify the script `start.sh` by changing the first line to look like the following:
  ```txt
  export FLASK_APP=pi4_qr_scanner.py
  ```
  
  Start the server:
  ```bash
  bash ./start.sh
  ```

From here on, just navigate to [http://<your-rip-ip/hostname>:5000](http://<your-rip-ip/hostname>:5000) to activate the cammera and scan the qr codes.