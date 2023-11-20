//Check url for anchors
window.onload = checkAnchor;

//Check anchors from url to go on a specific tab
function checkAnchor() {
    var anchor = window.location.hash.substring(1); //get anchor
    if (anchor) {
        openTab(anchor); //go to this tab
    }
    else{
        openTab("tab1"); //go to tab1
    }
}

/*Manage tab clicks*/
function openTab(tabId) {

    //Hide tabs content
    var tabContents = document.querySelectorAll('.tab-content');
    tabContents.forEach(function (tab) {
        tab.style.display = "none";
    });

    //Display the right content 
    var activeTab = document.getElementById(tabId+"_content");
    activeTab.style.display = "block";

    //Display video stream if current tab is 2
    if (tabId === "tab2"){
        fetch('/start_video').catch(error=>console.error("Error", error));
        //Get content from video_feed url
        document.getElementById("video-stream").src=video_stream_link;
    }
    //Else remove it, might put a default img later...
    else {
        fetch('/stop_video').catch(error=>console.error("Error", error));
        document.getElementById("video-stream").src="";
    }

    //Change url to delete the anchor
    history.replaceState({ tabId: tabId }, null, "/camera");
}

/*Manage submit button after photo is taken so a response appear in the page w/o reloading
using Ajax to do so, in other word this function is called before sending data forms to the app server*/
function submitPhotoForm() {
//Get form data
var photoFormData = $('#photoForm').serialize();

    //Ajax request with JQuery type Post
    $.ajax({
        type: 'POST',
        url: "/camera",
        data: photoFormData,
        success: function(response){ //'reponse' was created in flask app

            var paragraph = document.getElementById("resultMessageParagraph");
            //Display a response message
            if (response.success) {         
                paragraph.innerHTML = '<pre>' + response.message + '</pre>';                    
            }
            //Or display an error message
            else {
                paragraph.textContent = response.error; //error from flask
            }
        },
        error: function(error){ //error from AJAX
            //Display errors in the console
            console.log('Error from camera.html:', error);
            }
        });
}



/*Manage click on item photo list*/
function showPhotoModal(index){
    var modal = document.getElementById("photo_modal");//get modal
    var img = document.getElementById("img_modal"); //get image
    var caption = document.getElementById("caption_modal"); //get caption

    img.src = photo_modal_link + list_photos[index]["name"];//set img
    caption.innerHTML = list_photos[index]["name"]//set caption
    modal.style.display = "block"; //show modal
}

/*Quit modal in click on 'X'*/
function closeModal(){
    var modal = document.getElementById("photo_modal");//get modal
    modal.style.display = "none"; //modal disappear
}
//Quit the modal if the user clicks outside of it
window.onclick = function(event) {
    var modal = document.getElementById("photo_modal");
    if (event.target === modal) {
        modal.style.display = "none";
    }
};

