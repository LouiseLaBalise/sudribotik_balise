let NB_PAMIS = 6; //this will be automatcally update when clienter this page

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
    
    //Get the data sent
    const pami_infos = JSON.parse(event.data);

    //Get number of pamis
    let nb_pamis = Object.keys(pami_infos).length;
    if (nb_pamis>6){nb_pamis=6;} //Not more than 6 pamis
    NB_PAMIS = nb_pamis;

    //Modify the client page
    for (let i = 0; i < nb_pamis; i++){
        //Select pami id from html and set right data from python app.py

        //Set its tags number
        const num_aruco = document.getElementById("num_aruco_"+(i+1));
        num_aruco.innerText = pami_infos[i].tag;

        //Set its connection status
        const connection_status = document.getElementById("connection-status-icon_"+(i+1));
        /*if (pami_infos.connection){
            connection_status.src = "{{ url_for('static', filename='/icons/connected_64px.png')}}"
        }
        else{
            connection_status.src = "{{ url_for('static', filename='/icons/not_connected_64px.png')}}";
        }*/

        //Set its coordinate
        const pos_x = document.getElementById("pos_x_"+(i+1));
        pos_x.innerText = pami_infos[i].pos_x;

        const pos_y = document.getElementById("pos_y_"+(i+1));
        pos_y.innerText = pami_infos[i].pos_y;

        const pos_theta = document.getElementById("pos_theta_"+(i+1));
        pos_theta.innerText = pami_infos[i].pos_theta;
    }

};

//When page is closed the event no info will be generated
window.onbeforeunload = function() {
    eventSource.close();
};


/*Function called when gotPami form are submitted / Go button is pushed*/
function goPamiFormSubmit(event,pami_number){

    //Prevents the default behaviour of the browser submitting the form
    //it is inside a try because of when this funct get call by generalGoPamis() ther is no 'event'
    try{event.preventDefault();}
    catch (error){}

    //Get x and y position in the form and add it to a variable
    var gotoFormData = {
        number:pami_number, //Add pami's number in the data form
        goto_x:document.getElementById("goto_x_"+pami_number).value,
        goto_y:document.getElementById("goto_y_"+pami_number).value,
    }

    //Fetch request with Post method
    fetch("/pamis",{
        method: 'POST',
        headers : { 
            'Content-Type': 'application/json',
            'Accept': 'application/json'
           },
        body: JSON.stringify(gotoFormData), //data to be sent
    })
    .then(response => response.json())//wait for a response and parse it to json
    .then(data => {
        //Display response success into the console
        if (data.success){
            console.log(`Pami ${pami_number} sent to \
                        (${gotoFormData['goto_x']}, ${gotoFormData['goto_y']})`);
        }
        //Or display an error message
        else {
            console.log(data.error)//error from flask
        }
    })
}



/*Function called when general go is pushed*/
function generalGoPamis(){
    
    for (let i = 1; i <= NB_PAMIS; i++){
        goPamiFormSubmit(0,i)
    }
}



/*Function called when stop is pushed*/
function stopPami(num_pami){

    //Fetch request with Post method
    fetch('/stop_pami', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        },
        body: JSON.stringify({pami_number: num_pami }), //data to be sent
    })
    .then(response => response.json())
    .then(data => {
        //Display response success into the console
        if (data.success) {
            console.log('Pami', num_pami, "stopped.");
        }
        //Or display an error message
        else {
            console.log(data.error)//error from flask
        }
    })
}
/*Function called when general stop is pushed*/
function generalStopPamis(){
    for (let i = 1; i <= NB_PAMIS; i++){
        stopPami(i);
    }
}

//Add a listener on the toggle switch
/*Function called whenever the toggle switch for image reddressement is changed*/
function toggleRedressChanged(checkbox){
    //Get checkbox state
    var is_checked = checkbox.checked;
    
    //Fetch request with Post method
    fetch('/pami_redress_image', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        },
        body: JSON.stringify({redress: is_checked }), //data to be sent
    })
    .then(response => response.json())
    .then(data => {
        //Display response success into the console
        if (data.success) {
            if (data.redress) {
                console.log("Now image will be redressed.");
            }
            else{
                console.log("Stop to redress image");
            }
        }
        //Or display an error message
        else {
            console.log(data.error)//error from flask
        }
    })
}