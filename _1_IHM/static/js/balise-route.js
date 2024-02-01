//Check url for anchors
window.onload = checkAnchor;

/*Check anchors from url to go on a specific tab*/
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
    tabContents.forEach(function (content) {
        content.style.display = "none";
    });

    //Remove all 'current' and set z-index
    var tabs = document.querySelectorAll('.tab');
    var tab_index = 1;
    var z_index = 50;
    tabs.forEach(function(tab){
        tab.classList.remove("current");
        if ("tab"+tab_index !== tabId) {
            document.getElementById("tab"+tab_index).style.zIndex = z_index;
        }
        else{
            /*Set current z-index at 80 to be on top of the other tabs*/
            document.getElementById("tab"+tab_index).style.zIndex = 80;
        }
        tab_index++;
        z_index--;
    });

    //Display the right content
    var activeTab = document.getElementById(tabId+"_content");
    activeTab.style.display = "block";
    document.getElementById(tabId).classList.add("current");//put tab on class 'current'

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
    history.replaceState({ tabId: tabId }, null, "/balise");
}


/*Manage enabling/disabling inputs in advanced options*/
function toggleInput(checkbox, class_params){
    //Get checkbox state
    var is_checked = checkbox.checked;

    //Get all params by their class
    var params = document.getElementsByClassName(class_params);

    //Enable corresponding params
    if (is_checked){
        for (var i = 0; i < params.length; i++) {
            params[i].disabled = false;
        }
    }
    //Or disable corresponding params
    else {
        for (var i = 0; i < params.length; i++) {
            params[i].disabled = true;
        }
    }
}


/*Manage submit button after photo is taken so a response appear in the page w/o reloading
using Ajax to do so, in other word this function is called before sending data forms to the app server*/
function submitPhotoForm() {
//Get form data
var photoFormData = $('#photoForm').serialize();

    //Ajax request with JQuery type Post
    $.ajax({
        type: 'POST',
        url: "/balise",
        data: photoFormData,
        success: function(response){ //'reponse' was created in flask app

            var paragraph = document.getElementById("resultMessageParagraph");
            //Display a response message
            if (response.success) {
                paragraph.innerHTML = "<ul>"
                    +"<li>Photo prise avec succès. Elle est disponnible dans la galerie des photos.</li>"
                    +"<li>Photo d'origine : " + response.image_name + "</li>"
                    +"<li>Photo traitée : " +  response.processed_name + "</li>"
                    +"<li>Info : " + response.processed_info + "</li>"
                    +"</ul>";
            }
            //Or display an error message
            else {
                paragraph.textContent = response.error; //error from flask
            }
       },
        error: function(error){ //error from AJAX
            //Display errors in the console
            console.log('Error from balise-route.js:', error);
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

/*Quit modal if click on 'X'*/
function closeModal(){
    var modal = document.getElementById("photo_modal");//get modal
    modal.style.display = "none"; //modal disappear
}
/*Quit modal if user clicks outside of it*/
window.onclick = function(event) {
    var modal = document.getElementById("photo_modal");
    if (event.target === modal) {
        modal.style.display = "none";
    }
};

