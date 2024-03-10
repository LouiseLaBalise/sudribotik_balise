
/*Function called to update Louise general informations*/
function updateInformationLabel() {

    /*fetchdata on this route*/
    fetch('/get-home-information')
        .then(response => response.json())
        /*If there is data, diplayed them accordingly*/
        .then(data => {
            document.getElementById('temp_cpu').textContent = data.cpu_temp;
            document.getElementById('temp_gpu').textContent = data.gpu_temp;
            document.getElementById('storage_used').textContent = data.used_storage;
            document.getElementById('storage_max').textContent = data.max_storage;

            /*Case camera_check*/
            if (data.camera_check){
                document.getElementById('check_camera').src = "static/icons/connected_64px.png"
            }
            else{
                document.getElementById('check_camera').src = "static/icons/not_connected_64px.png"
            }

            document.getElementById('usb1').textContent = data.usb1;
            document.getElementById('usb2').textContent = data.usb2;
            document.getElementById('usb3').textContent = data.usb3;
            document.getElementById('usb4').textContent = data.usb4;
        })
        .catch(error => console.error('Error fetching the label value:', error));
}

updateInformationLabel()/*call for the first click on homepage*/
//Update the label every X ms
setInterval(updateInformationLabel, 2000);