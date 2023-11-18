import sys, cv2
sys.path.insert(1, "/home/ubuntu/Eurobot_2024") #add parent folder to python path -----------------------------------------------

from _3_TRAITEMENT_d_IMAGES import takePhoto #--------------------------------------------------------------------
from flask import Flask, render_template, request, jsonify, Response



app = Flask(__name__, static_url_path="/static")
TEMPLATES_AUTO_RELOAD = True #reload when template change
PHOTO_PATH = "EUROBOT_2024/_3_TRAITEMENT_d_IMAGES/medias/"
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

        try:
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
            #path_to_photo_taken = "" #SUPPRIMER CETTE LIGNE SUR LA RASPBERRY PI 4
            message = f"Photo prise avec succès.\nElle est disponnible dans le dossier {path_to_photo_taken} ou dans la galerie des photos.\nNom : {real_photo_name}\nQualité : {photo_quality}"

            #Create with response with success key True
            response = {'success': True, 'message': message}


        except Exception as e:
            #Create with response with success key False
            response = {'success': False, 'error': str(e)}


        #Get method B
        return jsonify(response)

    #Get method A
    return render_template("camera.html")


#Video for camera tab2
@app.route('/video_feed')
def video_feed():
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
def start_video():
    global streaming_mode
    streaming_mode=True
    return 'OK'
@app.route('/stop_video')
def stop_video():
    global streaming_mode
    streaming_mode=False
    return 'OK'

if __name__=="__main__":
    app.run(debug=True, host='0.0.0.0') #Web app accessible by any device on the network
