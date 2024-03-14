import sys
import os
import cv2
import traceback
import json
import numpy as np
import subprocess
from flask import Flask, render_template, request, jsonify, Response, send_from_directory, session

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)

sys.path.insert(1, FILE_PATH.split("_1_IHM")[0]) #add parent folder to python path
from init import network_manager
from _3_TRAITEMENT_d_IMAGES import (takePhoto, redressBoardUsingAruco, detectAruco, detectColor,
                                    ros_redressBoardUsingAruco, ros_detectAruco, ros_arucoCalc,
                                    ros_undistortImage)
from scripts.formatdata import formatBytes, formatSeconds


app = Flask(__name__)

TEMPLATES_AUTO_RELOAD = True #reload when template change
MEDIA_FOLDER_PATH = FILE_PATH.split("_1_IHM")[0]+"_3_TRAITEMENT_d_IMAGES/media/"
PHOTO_NAME_SUFFIXE = "_via_ihm"
PHOTO_EXTENSION = ".jpg"
streaming_mode=False #true when client open tab2 of beacon to see the camera view (2nd tab of beacon)
sse_pamis=True # [NOT IN USE] false when client quit pami page
pami_redress_image_before_detecting_aruco=False#true when toggle is switched
fail_redress = False #true if an attemp to redress went wrong
pseudo_session = {} #global dictionary to store some variable like a session
#Load all pamis tag at the start of the app, meaning if these have to change,
# you'll need to restart the app to updtae
configuration_FILEPATH = FILE_PATH.split("_1_IHM")[0]+"init/configuration.json"
with open (configuration_FILEPATH, "r") as f:
    config = json.load(f)
#Get transform matrix for redress here to be used later, cast it to np.array
transform_matrix = np.array(config["AUTO_TRANSFORM_MATRIX"])




#######################################################################################
#                                                                                     #
#                                       HOME                                          #
#                                                                                     #
#######################################################################################
    
@app.route('/')
@app.route('/accueil')
def home():
    """
    Homepage route
    """

    return render_template("home.html") #look for the home.html template in ./templates/




@app.route("/get-home-information")
def get_home_information():
    """
    Update Informations displayed on the homepage top-right square.
    """ 
    #Get CPU temperature    
    try:
        cpu_temp_string_bytes = subprocess.run(["cat", "/sys/class/thermal/thermal_zone0/temp"],
                                               stdout=subprocess.PIPE)
        cpu_temp_string = cpu_temp_string_bytes.stdout.decode('ascii') #cast bytes to str
        cpu_temp = int(cpu_temp_string)/1000 #round by 1 after dividing the temp
        cpu_temp = round(cpu_temp, 1)
        cpu_temp = str(cpu_temp)
    except Exception: cpu_temp = "-" #default
    
    #Get GPU temperature   
    try:
        gpu_temp_string_bytes = subprocess.run(["vcgencmd", "measure_temp"],
                                               stdout=subprocess.PIPE)
        gpu_temp_string = gpu_temp_string_bytes.stdout.decode('ascii') #cast bytes to str
        gpu_temp = gpu_temp_string[5:-2] #strip the temp
    except Exception: gpu_temp = "-" #default
    
    #Get used and max storage
    try:
        storage_string_bytes = subprocess.run(["df", "-h", "/"],
                                              stdout=subprocess.PIPE)
        storage_string = storage_string_bytes.stdout.decode('ascii').split() #cast bytes to str and remove spaces
        used_storage = storage_string[9]+'b' #get used storage
        max_storage = storage_string[8]+'b' #get max storage
    except Exception: 
        used_storage = "-" #default
        max_storage = "-"

    #Check for camera
    try:
        #We will check for camera just one time at the start of the page because we dont want to take a pic every seconds
        if "camera_checked" not in pseudo_session :
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2) #take a pic
            camera_check, frame = cap.read() #get return as camera check
            cap.release() #release camera
            pseudo_session["camera_checked"] = camera_check #store camera result

        #Get camera result previously stored
        else:
            camera_check = pseudo_session["camera_checked"]

    except Exception: 
        cap.release()
        camera_check = False #default

    #Check for all usb port
    try:
        usb_check_string_bytes = subprocess.run(["lsusb"],
                                                stdout=subprocess.PIPE)
        usb_check_string = usb_check_string_bytes.stdout.decode('ascii') #cast bytes to str
        
        #Loop through all devices
        all_usb_devices = [device.split(":")[2][5:-1] for device in usb_check_string.split("\n") if len(device)>4]
        
        #Get device name excluding those starting by the word 'Linux'
        device_names =  [name  for name in all_usb_devices if name.split()[0] != "Linux"]

        #Fill with '-' if there is less than 4 devices pluged in
        device_names = device_names[:4] + ["-"] * max(0, 4 - len(device_names))

    except Exception: device_names = ["-"]*4 #default

    

    return jsonify({"cpu_temp": cpu_temp, "gpu_temp": gpu_temp,
                    "used_storage": used_storage, "max_storage": max_storage,
                    "camera_check": camera_check,
                    "usb1": device_names[0], "usb2": device_names[1], "usb3": device_names[2], "usb4": device_names[3]})




#######################################################################################
#                                                                                     #
#                                       PAMIS                                         #
#                                                                                     #
#######################################################################################




def getPamiConnection(tag):
    """
    Get connection status of a pamis knowing its tag.
    
    Parameters:
        - tag (int): ArUco tag ID number of the pami. There is a special line
                    for pamis of the HMI in the configuration.json file.
    
    Returns:
        - bool: pami is connected or not.
    
    Note:
        The ip address is stored in the configuration.json file.
        The index of the tag in IHM_PAMI_IDS  give its IP address in IP_ADDRESS_PAMIS.
    """

    #Retrieve the ip address of the pami.
    index_pami_ihm = config["IHM_PAMI_IDS"].index(tag)

    #Get corresponding IP
    ip_pami = config["IP_ADDRESS_PAMIS"][index_pami_ihm]

    #Ping ip with a low wait time
    result_bytes = subprocess.run(["ping", "-c", "1", "-W", "0.13", ip_pami],
                                  stdout=subprocess.PIPE)
    result = result_bytes.stdout.decode("ascii").split() #get string result

    #If the result text got '0%' in it (from 0% packet loss) it means that the ping succeed.
    return "0%" in result


def getAllPamisPosition():
    """
    Take a photo and return position of all pamis tags inside,
    This function use ros functions to process faster.
    return a list of tuples (x, y, theta)
    """

    #Read an image from camera
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    ret,frame = cap.read()
    cap.release()

    #Get all pammi tag id for the debug/ihm configuration
    pami_ihm_tags = config["IHM_PAMI_IDS"]
    nb_pamis = len(pami_ihm_tags) #get number of pamis

    #Case no image
    if not ret : return [("N/A", "N/A", "N/A") for k in range(nb_pamis)]

    #Redress image
    global fail_redress
    if pami_redress_image_before_detecting_aruco:
        redress_success, frame = ros_redressBoardUsingAruco.redressImage(frame, transform_matrix,
                                                                         config["AUTO_REDRESS_SIZE"])
        #Case no redress
        if not redress_success : 
            print(f"Log [{os.times().elapsed}] - {FILE_NAME} : L'image n'a pas pu être redréssée.")
            fail_redress = True
        #If redress finally worked after some failures inform user just one time in log 
        if redress_success and fail_redress :
            print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Redressement de l'image réussi.")
            fail_redress = False
                

    #Detect Aruco with ros_detectaruco because it is the one used in match
    ret, all_aruco_corners, all_aruco_ids = ros_detectAruco.detectAruco(frame)

    #Case no Aruco
    if not ret : return [("N/A", "N/A", "N/A") for k in range(nb_pamis)]

    #Select only detected pami tags
    pami_ids_detected = list(set(all_aruco_ids) & set(pami_ihm_tags))

    #Calculate their position
    pamis_position = []
    for id in pami_ihm_tags :

        #If id has not been detected we add it to the list with no values
        if id not in pami_ids_detected:
            pamis_position.append(("N/A", "N/A", "N/A"))

        #Else just calculate its pos and angle then add it
        else:
            corner = all_aruco_corners[all_aruco_ids.index(id)]
            px, py = ros_arucoCalc.getCenterArucoTag(corner)
            angle = ros_arucoCalc.getAngle(corner, unit="degrees")
            pamis_position.append((str(px), str(py), str(angle)))
    
    return pamis_position





def generate_pamis_infos():
    """
    Generate json object with pamis informations inside.
    """
    while True: #generate pamis info continuously

        #Fetch all pami position on a single image
        all_pamis_pos = getAllPamisPosition()
        
        #Store them all
        pamis_informations = [{"tag":tag,
                               "connection":getPamiConnection(tag),
                               "pos_x":all_pamis_pos[k][0],
                               "pos_y":all_pamis_pos[k][1],
                               "pos_theta":all_pamis_pos[k][2] } for k,tag in enumerate(config["IHM_PAMI_IDS"])]

        #We need a generator to use SSE, data is jsoned in the process
        yield f"data: {json.dumps(pamis_informations)}\n\n"


@app.route('/sse_pamis')
def sse_pamis():
    """
    Server-Sent Event route needed to sent pamis information to client frequently.
    """
    #Return a Flask response using SSE data format
    return Response(generate_pamis_infos(), content_type='text/event-stream')




@app.route('/pami_redress_image', methods=['POST'])
def pami_redress_image():
    """
    Route used to change redress image state.
    """
    global pami_redress_image_before_detecting_aruco
    
    #Get checkbox state
    checkbox_state = request.json["redress"]
    
    #Send a command to the pami
    try :
        pami_redress_image_before_detecting_aruco = checkbox_state
        response={"success":True,
                  "redress":checkbox_state}


    except Exception as e:
        traceback.print_exc()
        #Create with response with success key False
        response = {'success': False, 'error': str(e)}

    #back to the client
    return jsonify(response)



@app.route('/pami_goto', methods=['POST'])
def pami_goto():
    """
    Pami goto() route. When called this call a goto function in the selected pami.
    """
    #Get pami's number
    pami_number = int(request.json["number"])

    #Get desired x and y position from the form
    desired_x = int(request.json["goto_x"])
    desired_y = int(request.json["goto_y"])
    
    #Send a command to the pami
    try :
        #goto() du script d'Alexandre <----------------------------------
        response={"success":True}


    except Exception as e:
        traceback.print_exc()
        #Create with response with success key False
        response = {'success': False, 'error': str(e)}

    #back to the client
    return jsonify(response)



@app.route('/pami_stop', methods=['POST'])
def stop_pami():
    """
    Route used to stop a pami.
    """    
    #Get which pami to stop
    pami_num_to_stop = request.json["pami_number"]
        
    #Send a command to the pami
    try :
        #stop() du script d'Alexandre <----------------------------------
        response={"success":True}


    except Exception as e:
        traceback.print_exc()
        #Create with response with success key False
        response = {'success': False, 'error': str(e)}

    #back to the client
    return jsonify(response)



@app.route('/pamis')
def pamis():
    """
    Pamis route.
    """        

    #Go to pamis page        
    return render_template("pamis.html")







#######################################################################################
#                                                                                     #
#                                      BEACON                                         #
#                                                                                     #
#######################################################################################





@app.route('/beacon', methods=['GET', 'POST'])
def beacon():
    """
    Beacon route.
    """
    #Get photo of all photos in PHOTO_PATH
    list_photos = makePhotoList(MEDIA_FOLDER_PATH)

    #Post method
    if request.method == 'POST':

        #Get form basic options
        photo_name = request.form["photo_name"]
        photo_name_suffixe = request.form.get("photo_name_suffixe", "no") == "checked"
        photo_tms = int(request.form["photo_tms"])
        photo_quality = int(request.form["photo_quality"])
        #If the toggle is not checked a default value is given to him ('no')
        #we just test if the value is similare with the one in the html form and createe a bool from it
        denoise = request.form.get("photo_denoising", "no") == "checked"

        #Get form advanced options
        undistort = request.form.get("undistort_yes_no", "no") == "checked"
        redress = request.form.get("redress_yes_no", "no") == "checked"
        detect_aruco = request.form.get("aruco_yes_no", "no") == "checked"
        detect_color = request.form.get("color_yes_no", "no") == "checked"
        detect_color_surface = request.form.get("color_surface_yes_no", "no") == "checked"

        #Create variables for the photo
        real_photo_name = "{}{}{}".format(photo_name,
        PHOTO_NAME_SUFFIXE if photo_name_suffixe else "",
        PHOTO_EXTENSION)
        processed_info=""#this will be displayed if the photo is taken

        try:
            #Take photo according to parameters
            path_to_photo_taken = takePhoto.takePhoto(name=real_photo_name,
                                                     tms=photo_tms,
                                                     quality=photo_quality,
                                                     denoise=denoise)


            #If photo is not taken (it will throw a NoneType error)
            if (not path_to_photo_taken):
                print(f"Log [{FILE_NAME}]: Problème lors de la prise de photo.")
                response = {'success': False, 'error': "Impossible de prendre une photo."}
                return jsonify(response)

            all_processed_images=[]#path to processed image list
            path_to_photo_processed=path_to_photo_taken

            #Compute image based on advanced options
            #1st undistort
            if undistort:
                #undistort will be the only option to use its ros function because no calibrate function on the hmi.
                #it takes time and effort for something effortlessly doable in the shell.
                frame = cv2.imread(path_to_photo_taken)
                ret, undistorted_frame = ros_undistortImage.undistortImage(frame,
                                                                           np.array(config["AUTO_K_DISTORTION"]),
                                                                           np.array(config["AUTO_D_DISTORTION"]),
                                                                           config["AUTO_ORIGINAL_PHOTO_SIZE"])
                if not ret:
                    processed_info+="Impossible de supprimer la distortion. Avez-vous calibré la caméra ?\n"
                
                else :
                    #We need to operate a savefile here because it is not implemented in the ros function
                    path_to_photo_processed = path_to_photo_processed.split(".jpg")[0] + "_undistorted.jpg"
                    cv2.imwrite(filename=path_to_photo_processed, img=undistorted_frame)
                    all_processed_images.append(path_to_photo_processed)

            #2nd redress
            if redress:
                corner_ids = (int(request.form["redress_id1"]),
                              int(request.form["redress_id2"]),
                              int(request.form["redress_id3"]),
                              int(request.form["redress_id4"]))
                ret, path_to_photo_processed = redressBoardUsingAruco.redressBoardUsingAruco(filename=path_to_photo_processed,
                                                                     corner_ids=corner_ids)
                all_processed_images.append(path_to_photo_processed)

                if not ret: processed_info+="L'image n'a pas pu être redréssée.\n"

            #3rd aruco tags
            if detect_aruco:
                ret, _, _, path_to_photo_processed = detectAruco.detectAruco(filename=path_to_photo_processed,
                                                                             drawId=True)
                all_processed_images.append(path_to_photo_processed)

                if not ret: processed_info+="Aucun ArUco n'a pu être détecté.\n"


            #4th color and surface
            if detect_color:
                hue = (int(request.form["hue_min"]), int(request.form["hue_max"]))
                saturation = (int(request.form["sat_min"]), int(request.form["sat_max"]))
                value = (int(request.form["val_min"]), int(request.form["val_max"]))

                if detect_color_surface:
                    color_minSurface = int(request.form["color_minSurface"])
                    color_maxSurface = int(request.form["color_maxSurface"])
                    ret, _, path_to_photo_processed = detectColor.colorDetection(filename=path_to_photo_processed,
                                                                 hue=hue,
                                                                 saturation=saturation,
                                                                 value=value,
                                                                 minSurface=color_minSurface,
                                                                 maxSurface=color_maxSurface,
                                                                 rectangled=True)
                    all_processed_images.append(path_to_photo_processed)
                else : #call function without surfaces
                    ret, _, path_to_photo_processed = detectColor.colorDetection(filename=path_to_photo_processed,
                                                                    hue=hue,
                                                                    saturation=saturation,
                                                                    value=value,
                                                                    rectangled=True)
                    all_processed_images.append(path_to_photo_processed)

                if not ret: processed_info+="La détection de couleur n'a pas été réalisée.\n"

            #Delete intermidiate files created except og and finale one
            if all_processed_images : all_processed_images.pop(-1)
            for file_path in all_processed_images:
                #test if the file exist and if its not the og photo and if its not the last photo processed
                if os.path.isfile(file_path) and file_path!=path_to_photo_taken and file_path!=path_to_photo_processed:
                    os.remove(file_path)

            #Create the message to display
            image_name = path_to_photo_taken.split('/')[-1] #get og photo
            processed_name = path_to_photo_processed.split('/')[-1] #get final process photo
            if image_name==processed_name: #if there is no process done to the photo
                processed_name='-'
            if not processed_info:#if ther is no info from image traitement
                processed_info="-"

            #Create response with success key True
            response = {'success': True, "image_name":image_name,
                                         "processed_name":processed_name,
                                         "processed_info":processed_info}


        except Exception as e:
            traceback.print_exc()
            #Create with response with success key False
            response = {'success': False, 'error': str(e)}

        #Get method after submit form for photo
        return jsonify(response)

    #Main get method
    return render_template("beacon.html", list_photos=list_photos, media_path=MEDIA_FOLDER_PATH)







@app.route('/video_stream')
def videoStream():
    """
    Video for beacon second tab.

    Note:
        When this function is called it returns a 'multipart/x-mixed-replace' in a HTTP response.
        Basicly it says to the server that it will receive a myriade of data, and that data will be 
        replaced by its next data a few after. Usally used to send a stream of video frames.
    """
    return Response(generateFrames(), mimetype='multipart/x-mixed-replace; boundary=frame') #boundary=frame to delimitate each piece of data sent (see generate_frame())


def generateFrames():
    """
    Generator of frames from video.
    """
    video_capture = cv2.VideoCapture(0, cv2.CAP_V4L2) #Open camera for video capturing

    while streaming_mode and video_capture:
        ret,frame=video_capture.read() #read an image from camera
        if not ret:continue
        ret, jpeg = cv2.imencode('.jpg', frame) #convert image in jpg
        frame=jpeg.tobytes() #convert jpg in bytes

        #Use of yield to returns actual value of the image and stop the function, keep its state and get back to it when
        #function is called again. note that it returns bytes
        yield (b'--frame\r\n' #'frame' is the delimiter of each piece of data
               b'Content-Type: image/jpg\r\n\r\n' + frame + b'\r\n')#We tell the client that we sent jpeg img
    
    if video_capture : video_capture.release() #free video capturing if not in streamin mode anymore



@app.route('/start_video')
def startVideoStream():
    """
    This route is used to start video stream 
    from the client side when user enter the second tab.
    """
    global streaming_mode
    streaming_mode=True
    return 'OK'

@app.route('/stop_video')
def stopVideoStream():
    """
    This route is used to stop video stream 
    from the client side when user quit the second tab.
    """
    global streaming_mode
    streaming_mode=False
    return 'OK'


def makePhotoList(path):
    """
    Make a list of all photo present in PHOTO_PATH. Each item contain the name,
    the size and the date & hour.
    
    Parameters:
        - path (str) : path to look for photos.
    
    Returns:
        - list of dict : like [{"name":kyoto.jpg, "size":3200, "unit":KB, "date":26 nov. 2022, "hour": 21:34:07}]
    """

    photos_list = []

    #Sort list by time: most recent is first
    photos = sorted(os.scandir(path), key=lambda photo: photo.stat().st_mtime, reverse=True)

    for item in photos:
        #eject hidden files
        if item.name[0] == "." : continue
        name = item.name
        size, unit = formatBytes(item.stat().st_size)
        date, hour = formatSeconds(item.stat().st_mtime)
        photos_list.append({"name": name, "size":size, "size_unit":unit, "hour": hour, "date":date})


    return photos_list



@app.route("/gallery/<path:filename>")
def getPhotoModal(filename):
    """
    Route called when click on the list of photos in the gallery
    It opens a modal window with the photo.
    """
    #Return photo with the correct filename in the photo folder
    return send_from_directory(MEDIA_FOLDER_PATH, filename)





if __name__=="__main__":
    #host=0.0.0.0 -> Web app accessible by any device on the same network
    #port=5024 -> Port to access web app
    app.run(debug=True, host="0.0.0.0",port=5024)







