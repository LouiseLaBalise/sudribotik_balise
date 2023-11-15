from flask import Flask, render_template, request, jsonify

app = Flask(__name__, static_url_path="/static")
TEMPLATES_AUTO_RELOAD = True #reload when template change
PHOTO_PATH = "EUROBOT_2024/1_IHM/photos/"
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
        try:
            #Get form options
            photo_name = request.form["photo_name"]
            photo_name_suffixe = request.form["photo_name_suffixe"]
            photo_tms = request.form["photo_tms"]
            photo_quality = request.form["photo_quality"]

            #Create variables for the photo
            real_photo_name = "{}{}{}".format(photo_name, 
            PHOTO_NAME_SUFFIXE if photo_name_suffixe else "", 
            PHOTO_EXTENSION)
            path_to_photo = PHOTO_PATH

            #Create message to display
            message = f"Photo prise avec succès.\nElle est disponnible dans le dossier {path_to_photo} ou dans la galerie des photos.\nNom : {real_photo_name}\nQualité : {photo_quality}"

            #Create with response with success key True
            response = {'success': True, 'message': message}

            print(message)
    
        except Exception as e:
            #Create with response with success key False
            response = {'success': False, 'error': str(e)}


        #Get method B
        return jsonify(response)

    #Get method A
    return render_template("camera.html")


if __name__=="__main__":
    app.run(debug=True, host='0.0.0.0') #Web app accessible by any device on the network
