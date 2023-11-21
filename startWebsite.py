from pickle import NONE
from flask import Flask, render_template, redirect, url_for, request
from flask_cors import CORS, cross_origin
import numpy as np
import logging
from main import robotType

# Do some definitions etc..

app = Flask(__name__)       # creating flask app
cors = CORS(app)            # needed for cross origin (to share data from website to flask app)

app.config['CORS_HEADERS'] = 'Content-Type'

global websiteInformation   # define global var as dict with important inputs from the website
websiteInformation = {'mode': 'off','xCoord': 0, 'yCoord': 0, 'zCoord': 0,
    'alphaPlus': 0, 'betaPlus': 0, 'gammaPlus': 0,
    'alphaMinus': 0, 'betaMinus': 0, 'gammaMinus': 0 }

log = logging.getLogger('werkzeug') # keep back terminal output
log.setLevel(logging.ERROR)         # keep back terminal output

# start with defining websites

# Home Website
@app.route("/", methods=['GET', 'POST'])    # routing (set link, if directly called in html -> methods needed)
def home():                                 # definde function for website
    websiteInformation['mode'] = 'stop'     # set current mode
    print(websiteInformation)
    if request.method == 'POST':            # check post requests
   
        if request.form['btn'] == 'Demo Programme':         # In home.html are inputs defined as submit with 'name= "btn"'.
            return redirect(url_for('demo'))                # Hence request.form is 'btn'. The button value is "Demo Programme"
                                                            # so the input is checking the value. After this, there is a redirect
        elif request.form['btn'] == 'Manuelle Steuerung':   # to demo - function (is later in the code).
            return redirect(url_for('steuerung'))           # redirect to steuerung function

        elif request.form['btn'] == 'Erweiterte Optionen':
            return redirect(url_for('optionen'))            # redirect to optionen function
    
    return render_template('home.html')     # if no button pressed -> show home.html

# Demo Website
@app.route("/Demo")                             # routing (no direct call in html, so no methods)
def demo():                                     # define function           
    websiteInformation['mode'] = 'demo'         # set current mode to demo
    print(websiteInformation)
    if request.method == 'POST':                # check if any button pressed
        if request.form['btn'] == 'Demo Programme':
            return redirect(url_for('demo'))

        elif request.form['btn'] == 'Manuelle Steuerung':
            return redirect(url_for('steuerung'))

        elif request.form['btn'] == 'Stop':
            return redirect(url_for('home'))

        elif request.form['btn'] == 'Erweiterte Optionen':
            return redirect(url_for('optionen'))

    else:
        return render_template('demo.html')     # if not button pressed, show demo.html
    

# Steuerung Website
@app.route("/Steuerung", methods=['GET', 'POST'])       # routing -> Steuerung get called in html file -> methods needed
def steuerung():                                        # define function 
    websiteInformation['mode'] = 'manual'               # set current mode to steuerung

    if request.method == 'POST':
        if request.form['btn'] == 'Demo Programme':
            return redirect(url_for('demo'))

        elif request.form['btn'] == 'Manuelle Steuerung':
            return redirect(url_for('steuerung'))

        elif request.form['btn'] == 'Stop':
            return render_template('home.html')

        elif request.form['btn'] == 'Erweiterte Optionen':
            return redirect(url_for('optionen'))
                    
    else:
        if robotType == 'delta':
            return render_template('steuerung_delta.html')     # without buttons for rotations
        elif robotType == 'quattro':
            return render_template('steuerung_quattro.html')    # with buttons for gamma - rotations
        elif robotType == '6rus':
            return render_template('steuerung_6rus.html')      # with buttons for all 3 rotations
        else:
            return render_template('steuerung_delta.html')

# extend Steuerungs- websites
# the next functions are routed as "/Steuerung/inputs/...". There is no graphical website behind this
# routes, but they exist for data exchange between website and flask app. The Structure is similar to 
# graphical website, but there is no redirct to an HTML.

@app.route("/Steuerung/inputs/LJS", methods=['GET', 'POST'])    # Define route for Left JoyStick input
@cross_origin()                                                 # allow data exchange via fetch methode
def steuerungInputsLJS():                                       # define function

    data = request.get_json(force=True)                          # recive data from fetch
    websiteInformation['xCoord'] = (data['distance'] * np.cos(data['radian']))/100  # do some calculations
    websiteInformation['yCoord'] = (data['distance'] * np.sin(data['radian']))/100  # and write to webSiteInformation
    print(websiteInformation)
    return NONE # there is no return so it is None

@app.route("/Steuerung/inputs/RJS", methods=['GET', 'POST'])    # Right Joystick is similar to LJS
@cross_origin()
def steuerungInputsRJS():

    data = request.get_json(force=True)
    websiteInformation['zCoord'] = (data['distance'] * np.sin(data['radian']))/100 # only one calculation
    print(websiteInformation)
    return NONE

@app.route("/Steuerung/inputs/AP", methods=['GET', 'POST']) # now we have inputs from our rotation buttons if these
@cross_origin()                                             # buttons existing. This is for positiv alpha rotation.
                                                            # Similar to joystick inputs, just without calcs
def steuerungInputsAP():

    data = request.get_json(force=True)
    websiteInformation['alphaPlus'] = data
    #print(websiteInformation)
    return NONE

@app.route("/Steuerung/inputs/AM", methods=['GET', 'POST']) # negative alpha rotation
@cross_origin()
def steuerungInputsAM():

    data = request.get_json(force=True)
    websiteInformation['alphaMinus'] = data
    #print(websiteInformation)
    return NONE

@app.route("/Steuerung/inputs/BP", methods=['GET', 'POST']) # positiv beta rotation
@cross_origin()
def steuerungInputsBP():

    data = request.get_json(force=True)
    websiteInformation['betaPlus'] = data
    #print(websiteInformation)
    return NONE

@app.route("/Steuerung/inputs/BM", methods=['GET', 'POST']) # negative beta rotation
@cross_origin()
def steuerungInputsBM():

    data = request.get_json(force=True)
    websiteInformation['betaMinus'] = data
    #print(websiteInformation)
    return NONE

@app.route("/Steuerung/inputs/CP", methods=['GET', 'POST']) # positiv gamma rotation
@cross_origin()
def steuerungInputsCP():

    data = request.get_json(force=True)
    websiteInformation['gammaPlus'] = data
    print(websiteInformation)
    return NONE

@app.route("/Steuerung/inputs/CM", methods=['GET', 'POST']) # negativ gamma rotation
@cross_origin()
def steuerungInputsCM():

    data = request.get_json(force=True)
    websiteInformation['gammaMinus'] = data
    #print(websiteInformation)
    return NONE

# back to the normal websites, now we have options

@app.route("/Optionen", methods=['GET', 'POST']) # Optionen get called in HTML -> methods needed
def optionen():
    if request.method == 'POST':
        if request.form['btn'] == 'Zurück':
            return redirect(url_for('home'))

        if request.form['btn'] == 'Homing':    
            return redirect(url_for('homing'))    

        elif request.form['btn'] == 'Motoren stromlos':
            return redirect(url_for('off'))
    
    else:
        return render_template('optionen.html')


@app.route("/Homing") # if homing get called, there is an alert, the mode is set and the options site is called
def homing():
    websiteInformation['mode'] = 'calibrate'
    print(websiteInformation)
    return redirect(url_for('optionen'))
    

@app.route("/Off")  # turning motors off
def off():
    websiteInformation['mode'] = 'off'
    print(websiteInformation)
    if request.method == 'POST':
        if request.form['btn'] == 'Zurück':
            return redirect(url_for('home'))
        
        elif request.form['btn'] == 'Roboter Homing':
            return redirect(url_for('home'))

        elif request.form['btn'] == 'Motoren stromlos':
            return redirect(url_for('off'))

    else:
        return render_template("offopt.html")



if __name__ == '__main__':
    app.run(debug=True, port=5000, host='0.0.0.0')
