import sys
import os
import cv2
from flask import Flask, render_template, request, Response, send_from_directory

#sys.path.insert(1, "/home/ubuntu/Eurobot_2024") #add parent folder to python path -----------------------------------------------
#from _3_TRAITEMENT_d_IMAGES import takePhoto #--------------------------------------------------------------------

from scripts.formatdata import formatBytes, formatSeconds




app = Flask(__name__)
TEMPLATES_AUTO_RELOAD = True #reload when template change
MEDIA_FOLDER_PATH = "/home/rayane/Royone/Inge3M/projet/raspberry_pi_4/_3_TRAITEMENT_d_IMAGES/media/"#"/home/ubuntu/EUROBOT_2024/_3_TRAITEMENT_d_IMAGES/medias/"
PHOTO_NAME_SUFFIXE = "_via_ihm"
PHOTO_EXTENSION = ".jpg"
streaming_mode=False #true when client open camera to see the view (2nd tab of camera)

#Homepage route
@app.route('/')
def index():
    return render_template('index.html') #look for the index.html template in ./templates/

#Camera route
@app.route('/camera', methods=['GET', 'POST'])
def camera():

    #Get photo of all photos in PHOTO_PATH
    list_photos = makePhotoList(MEDIA_FOLDER_PATH)

    #Post method
    if request.method == 'POST':

        #Get form options
        photo_name = request.form["photo_name"]
        photo_name_suffixe = request.form["photo_name_suffixe"]
        photo_tms = int(request.form["photo_tms"])
        photo_quality = int(request.form["photo_quality"])

        #Create variables for the photo
        real_photo_name = "{}{}{}".format(photo_name,
        PHOTO_NAME_SUFFIXE if photo_name_suffixe else "",
        PHOTO_EXTENSION)

        """try:
            #Take photo according to parameters
            path_to_photo_taken = takePhoto.takePhoto(name=real_photo_name,
                                                     tms=photo_tms,
                                                     quality=photo_quality)

            #Update real_photo_name because if the same user enter the same
            #name the takePhoto() function will add an index to it
            real_photo_name = path_to_photo_taken.split('/')[-1]

            #If photo is not taken (it will throw a NoneType error)
            if (not real_photo_name):
                print("Problème lors de la prise de photo.")

            #Create the message to display
            path_to_photo_taken = "" #COMMENTER CETTE LIGNE SUR LA RASPBERRY PI 4
            message = f"Photo prise avec succès.\nElle est disponnible dans le dossier {path_to_photo_taken} ou dans la galerie des photos.\nNom : {real_photo_name}\nQualité : {photo_quality}"

            #Create with response with success key True
            response = {'success': True, 'message': message}


        except Exception as e:
            #Create with response with success key False
            response = {'success': False, 'error': str(e)}


        #Get method after submit form for photo
        return jsonify(response)#"""

    #Main get method 
    return render_template("camera.html", list_photos=list_photos, media_path=MEDIA_FOLDER_PATH)


#Video for camera tab2
@app.route('/video_stream')
def videoStream():
    #When this function is called it returns a 'multipart/x-mixed-replace' in a HTTP response.
    # Basicly it says to the server that it will receive a myriade of data,
    # and that data will be replaced by its next data a few after. Usally used to send a stream of video frames

    return Response(generateFrames(), mimetype='multipart/x-mixed-replace; boundary=frame') #boundary=frame to delimitate each piece of data sent (see generate_frame())

#Generator of frames from video
def generateFrames():
    video_capture = cv2.VideoCapture(0, cv2.CAP_V4L2) #Open camera for video capturing

    while streaming_mode:
        ret,frame=video_capture.read() #read an image from camera
        if not ret:continue
        ret, jpeg = cv2.imencode('.jpg', frame) #convert image in jpg
        frame=jpeg.tobytes() #convert jpg in bytes

        #Use of yield to returns actual value of the image en stop the function, keep its state and get back to it when
        #function is called again. note that it returns bytes
        yield (b'--frame\r\n' #'frame' is the delimiter of each piece of data
               b'Content-Type: image/jpg\r\n\r\n' + frame + b'\r\n')#We tell the client that we sent jpeg img
    video_capture.release() #free video capturing if not in streamin mode anymore

#These routes are used to start and stop video stream from the client side when user enter or quit a tab
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
    photo_list = []
    for item in os.scandir(MEDIA_FOLDER_PATH):
        name = item.name
        size, unit = formatBytes(item.stat().st_size)
        date, hour = formatSeconds(item.stat().st_mtime)
        photo_list.append({"name": name, "size":size, "size_unit":unit, "hour": hour, "date":date})

    return photo_list


"""Route called when click on the list of photos in the gallery
It opens a modal window with the photo.
"""
@app.route("/gallery/<path:filename>")
def getPhotoModal(filename):
    #Return photo with the correct filename in the photo folder
    return send_from_directory(MEDIA_FOLDER_PATH, filename)

    





if __name__=="__main__":
    app.run(debug=True, host='0.0.0.0') #Web app accessible by any device on the network
