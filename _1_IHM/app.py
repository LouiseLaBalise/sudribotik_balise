import sys
import os
import cv2
import traceback
import time, json
from flask import Flask, render_template, request, jsonify, Response, send_from_directory

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)

sys.path.insert(1, FILE_PATH.split("_1_IHM")[0]) #add parent folder to python path
from init import get_hotspot_ip_address
from _3_TRAITEMENT_d_IMAGES import (takePhoto, redressBoardUsingAruco, detectAruco, detectColor,
                                    ros_redressBoardUsingAruco, ros_detectAruco, ros_arucoCalc)
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
#Load all pamis tag at the start of the app, meaning if these have to change,
# you'll need to restart the app to updtae
configuration_FILEPATH = FILE_PATH.split("_1_IHM")[0]+"init/configuration.json"
with open (configuration_FILEPATH, "r") as f:
    config = json.load(f)["IHM_PAMI_IDS"]
    

"""Homepage route"""
@app.route('/')
def index():
    return render_template('index.html') #look for the index.html template in ./templates/






#######################################################################################
#                                                                                     #
#                                       PAMIS                                         #
#                                                                                     #
#######################################################################################



"""Returns whether a Pami is connected or not, based on their tag"""
def getPamiConnection(tag):
    return False


"""Take a photo and return position of all pamis tags inside,
This function use ros functions to process faster.
 return a list of tuples (x, y, theta)"""
def getAllPamisPosition():

    #Read an image from camera
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    ret,frame = cap.read()
    cap.release()

    nb_pamis = len(config) #get number of pamis

    #Case no image
    if not ret : return [("N/A", "N/A", "N/A") for k in range(nb_pamis)]

    #Redress image
    global fail_redress
    if pami_redress_image_before_detecting_aruco:
        redress_success, frame = ros_redressBoardUsingAruco.redressImage(frame, config["TRANSFORM_MATRIX"])
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
    pami_ids_detected = list(set(all_aruco_ids) & set(config))

    #Calculate their position
    pamis_position = []
    for id in config :

        #If id has not been detected we add it to the list with no values
        if id not in pami_ids_detected:
            pamis_position.append(("N/A", "N/A", "N/A"))

        #Else just calculate its pos and angle then add it
        else:
            corner = all_aruco_corners[all_aruco_ids.index(id)]
            px, py = ros_arucoCalc.getCenterArucoTag(corner)
            angle = ros_arucoCalc.getAngle(corner)
            pamis_position.append((str(px), str(py), str(angle)))
    
    return pamis_position



"""Generate json object with pamis informations inside"""
def generate_pamis_infos():
    while True: #generate pamis info continuously

        #Fetch all pami position on a single image
        all_pamis_pos = getAllPamisPosition()
        
        #Store them all
        pamis_informations = [{"tag":tag,
                               "connection":getPamiConnection(tag),
                               "pos_x":all_pamis_pos[k][0],
                               "pos_y":all_pamis_pos[k][1],
                               "pos_theta":all_pamis_pos[k][2] } for k,tag in enumerate(config)]

        #We need a generator to use SSE, data is jsoned in the process
        yield f"data: {json.dumps(pamis_informations)}\n\n"


"""Server-Sent Event route needed to sent pamis information to client frequently"""
@app.route('/sse_pamis')
def sse_pamis():
    #return a Flask response using SSE data format
    return Response(generate_pamis_infos(), content_type='text/event-stream')


"""route used to stop a pami"""
@app.route('/stop_pami', methods=['POST'])
def stop_pami():
    
    #Get which pami to stop
    pami_num_to_stop = request.json["pami_number"]
        
    #Send a command to the pami
    try :
        #stop() du script d'Alexandre
        response={"success":True}


    except Exception as e:
        traceback.print_exc()
        #Create with response with success key False
        response = {'success': False, 'error': str(e)}

    #back to the client
    return jsonify(response)

"""route used to change redress image state"""
@app.route('/pami_redress_image', methods=['POST'])
def pami_redress_image():
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


"""pamis route"""
@app.route('/pamis', methods=['GET', 'POST'])
def pamis():

    #Post method
    if request.method == 'POST':

        #Get pami's number
        pami_number = request.json["number"]

        #Get desired x and y position from the form
        desired_x = request.json["goto_x"]
        desired_y = request.json["goto_y"]
        
        #Send a command to the pami
        try :
            #goto() du script d'Alexandre
            response={"success":True}


        except Exception as e:
            traceback.print_exc()
            #Create with response with success key False
            response = {'success': False, 'error': str(e)}

        #back to the client
        return jsonify(response)

        

    #Go to pamis page        
    return render_template("pamis.html")







#######################################################################################
#                                                                                     #
#                                      BEACON                                         #
#                                                                                     #
#######################################################################################




"""beacon route"""
@app.route('/beacon', methods=['GET', 'POST'])
def beacon():

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
            path_to_photo_taken = ""
            path_to_photo_taken = takePhoto.takePhoto(name=real_photo_name,
                                                     tms=photo_tms,
                                                     quality=photo_quality,
                                                     denoise=denoise)


            #If photo is not taken (it will throw a NoneType error)
            if (not path_to_photo_taken):
                print(f"Log [{FILE_NAME}]: Problème lors de la prise de photo.")
                return

            all_processed_images=[]#path to processed image list
            path_to_photo_processed=path_to_photo_taken

            #Compute image based on advanced options
            #1st aruco tags
            if detect_aruco:
                ret, _, _, path_to_photo_processed = detectAruco.detectAruco(filename=path_to_photo_processed,
                                                             drawId=True)
                all_processed_images.append(path_to_photo_processed)

                if not ret: processed_info+="Aucun ArUco n'a pu être détecté.\n"

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

            #3rd color and surface
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

            #Delete files created except og and finale one
            if all_processed_images : all_processed_images.pop(-1)
            for file_path in all_processed_images:
                if os.path.isfile(file_path):#test if the file exist quand même
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







"""Video for beacon tab2"""
@app.route('/video_stream')
def videoStream():
    #When this function is called it returns a 'multipart/x-mixed-replace' in a HTTP response.
    # Basicly it says to the server that it will receive a myriade of data,
    # and that data will be replaced by its next data a few after. Usally used to send a stream of video frames

    return Response(generateFrames(), mimetype='multipart/x-mixed-replace; boundary=frame') #boundary=frame to delimitate each piece of data sent (see generate_frame())


"""Generator of frames from video"""
def generateFrames():
    video_capture = cv2.VideoCapture(0, cv2.CAP_V4L2) #Open camera for video capturing

    while streaming_mode:
        ret,frame=video_capture.read() #read an image from camera
        if not ret:continue
        ret, jpeg = cv2.imencode('.jpg', frame) #convert image in jpg
        frame=jpeg.tobytes() #convert jpg in bytes

        #Use of yield to returns actual value of the image and stop the function, keep its state and get back to it when
        #function is called again. note that it returns bytes
        yield (b'--frame\r\n' #'frame' is the delimiter of each piece of data
               b'Content-Type: image/jpg\r\n\r\n' + frame + b'\r\n')#We tell the client that we sent jpeg img
    video_capture.release() #free video capturing if not in streamin mode anymore


"""These routes are used to start and stop video stream 
from the client side when user enter or quit a tab"""
@app.route('/start_video')
def startVideoStream():
    global streaming_mode
    streaming_mode=True
    return 'OK'
@app.route('/stop_video')
def stopVideoStream():
    global streaming_mode
    streaming_mode=False
    return 'OK'

"""Make a list of all photo present in PHOTO_PATH
arg : path:str
Return : list of dict -> [{"name":kyoto.jpg, "size":150, "unit":MB, "date":25 nov. 2022, "hour": 21:54:07}]"""
def makePhotoList(path:str):
    photos_list = []

    #Sort list by time: most recent is first
    photos = sorted(os.scandir(MEDIA_FOLDER_PATH), key=lambda photo: photo.stat().st_mtime, reverse=True)
    for item in photos:
        #eject hidden files
        if item.name[0] == "." : continue
        name = item.name
        size, unit = formatBytes(item.stat().st_size)
        date, hour = formatSeconds(item.stat().st_mtime)
        photos_list.append({"name": name, "size":size, "size_unit":unit, "hour": hour, "date":date})


    return photos_list


"""Route called when click on the list of photos in the gallery
It opens a modal window with the photo.
"""
@app.route("/gallery/<path:filename>")
def getPhotoModal(filename):
    #Return photo with the correct filename in the photo folder
    return send_from_directory(MEDIA_FOLDER_PATH, filename)





if __name__=="__main__":
    #Check for wlan0 in case of a hotspot is on
    host_ip = get_hotspot_ip_address.get_ip()

    #host=0.0.0.0 -> Web app accessible by any device on the same network
    #port=5024 -> Port to access web app
    app.run(debug=True, host=f"{host_ip}",port=5024)







