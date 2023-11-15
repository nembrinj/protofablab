
To set up and start the scanner software on your aiy vision pi do the following.



1. First, clone the github repo.

2. Install the python dependencies :

    ```bash
   pip install -r requirements.txt
   ```

4. Navigate to qr scanner folder :

   ```bash
   cd ./qr_scanner
   ```

5. Open up port 5000 on your aiy vision pi : 

   ```bash
   ufw allow 5000
   ```

6. Set the FLASK_APP environment variable to aiy_qr_scanner.py : 

   ```bash
   export FLASK_APP=aiy_qr_scanner.py
   ```

7. Start the flask server :

   ```bash
   flask run --host 0.0.0.0
   ```
From here on you can just navigate to http://<your-rip-ip>:5000 to activate the cammera and scan qrcodes 
