import sys
sys.path.insert(1, "/home/ubuntu/Eurobot_2024") #add parent folder to python path

from _3_TRAITEMENT_d_IMAGES import takePhoto
from flask import Flask, render_template, request, jsonify


app = Flask(__name__, static_url_path="/static")
TEMPLATES_AUTO_RELOAD = True #reload when template change
PHOTO_PATH = "EUROBOT_2024/_3_TRAITEMENT_d_IMAGES/medias/"
PHOTO_NAME_SUFFIXE = "_via_ihm"
PHOTO_EXTENSION = ".jpg"

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

            #Create message to display
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


if __name__=="__main__":
    app.run(debug=True, host='0.0.0.0') #Web app accessible by any device on the network
