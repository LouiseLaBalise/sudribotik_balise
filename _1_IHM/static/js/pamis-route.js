
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