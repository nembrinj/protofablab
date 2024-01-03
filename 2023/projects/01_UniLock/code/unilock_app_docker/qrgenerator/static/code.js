function lockDoor() {
    fetch('/close_door/')
    .then(response => response.json())
    .then(data => {
        document.getElementById("welcome-msg").style.display = 'none';
        document.getElementById("closeImgContainer").style.display = 'flex';
        document.getElementById("buttons").style.display = 'none';

        setTimeout(() => {
            // document.getElementById("closeImgContainer").innerHTML = `<img src="{% static 'closed_info.png' %}">`;
            document.getElementById("closeImgContainer").style.display = 'none';
            document.getElementById("welcome-msg").style.display = 'flex';
            document.getElementById("buttons").style.display = 'block';
            location.reload();
        }, 2500); 
    })
    .catch(error => {

    });
}

function generateQR() {
    document.getElementById("welcome-msg").style.display = 'none';
    document.getElementById("middle-text-section").style.display = 'flex';
    document.getElementById("qrCodeContainer").style.display = 'flex';
    document.getElementById("buttons").style.display = 'none';

    // Call the Django view to generate or load a QR code
    fetch('/generate_qr/')
        .then(response => response.json())
        .then(data => {
            // Display the QR code
            document.getElementById("qrCodeContainer").innerHTML = `<img id='qr-img' src="data:image/png;base64,${data.img_data}" alt="QR Code" style="width:100%;height:100%;">`;

            // Enable the button after the remaining time
            setTimeout(() => {
                document.getElementById("middle-text-section").style.display = 'none';
                document.getElementById("qrCodeContainer").style.display = 'none';
                document.getElementById("welcome-msg").style.display = 'flex';
                document.getElementById("buttons").style.display = 'block';
                location.reload();
            }, data.remaining_time * 1000);

            // Update remaining time
            let remainingTimeSpan = document.getElementById("remainingTime");
            let remainingTime = data.remaining_time;

            function updateRemainingTime() {
                remainingTimeSpan.innerText = " " + remainingTime--;
                if (remainingTime >= 0) {
                    setTimeout(updateRemainingTime, 1000);
                }
            }

            

            fetch('/turn_on_camera/')
            .then(response => response.json())
            .then(data => {
                if(data.message == "Camera failed to turn on"){
                    alert(" wrong qrcode ")
                    /*
                    document.getElementById("welcome-msg").style.display = 'none';
                    document.getElementById("closeImgContainer").style.display = 'flex';
                    document.getElementById("buttons").style.display = 'none';
                    setTimeout(() => {
                        // document.getElementById("closeImgContainer").innerHTML = `<img src="{% static 'closed_info.png' %}">`;
                        document.getElementById("closeImgContainer").style.display = 'none';
                        document.getElementById("welcome-msg").style.display = 'flex';
                        document.getElementById("buttons").style.display = 'block';
                        location.reload();
                    }, 20000); 
                    */
                }else{
                    alert(" right qrcode ")

                }
            }).catch(err => console.log(err))

            updateRemainingTime();
        });
}