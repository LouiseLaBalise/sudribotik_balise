
console.log("bonjour")

//When a tab is clicked this function is called
function showTab(tabId){

    var a = document.getElementById('a');
    a.style.color = 'red';

    //Hide all tabs
    var tabs = document.querySelectorAll(".tab-content");

    tabs.forEach((tab) => {
        tab.classList.remove('active')
    }) 

    //Show only the tab clicked
    var activeTab = document.getElementById(tabId);
    activeTab.classList.add("active");
}


document.querySelectorAll("[data-tab]").forEach(tab=>tab.addEventListener("click",showTab(tab.getAttribute("data-tab"))))