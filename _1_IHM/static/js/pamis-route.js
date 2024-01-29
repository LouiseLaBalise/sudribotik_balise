
//Check url for anchors every time page is reloaded
window.onload = checkAnchor;

/*Check anchors from url to go on a specific tab*/
function checkAnchor() {
    var anchor = window.location.hash.substring(1); //get anchor
    if (anchor) {
        openTab(anchor); //go to this tab
    }
    //without anchore default tab is opened
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
}



//Create an event on /sse_pamis route
const eventSource = new EventSource("/sse_pamis");

//When this event receive a message update the html page with the message received
eventSource.onmessage = function(event){
    console.log("J'ai recu : ", event.data);
};

//When page is closed the event and then no info is generated
window.onbeforeunload = function() {
    eventSource.close();
};


/*Function called to update Pamis infos
function updatePamis(){

    //AJAX request      
    $.ajax({
        type: "POST",
        url: "/pamis",
        data: pamis_data,
        success: function(data){
            $("balise_test").text(data.connection);
            //Loop over all pamis
            /*for (let i = 0; i < 6; i++){
                //Select pami id from html and set right data from python app.py
                $("#num_aruco_"+i).text(data.num_aruco);
                $("#connection_"+i).text(data.connection);
                $("#position_"+i).text(data.position_x);
            }
        }
    });
    
}
console.log("cvec");*/
/*function updatePamis(){

    //AJAX request
    //for (let i = 0; i < 6; i++) {   
        $.get("/pamis", function(data){
            //Select pami id from html and set right data from python app.py
            $("#balise_test").text(data.connection);
        });
    //}
    
}

//Call this function every 1 second
setInterval(updatePamis, 1000);*/