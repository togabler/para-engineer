import time
import RPi.GPIO as GPIO
from Robot import Robot

from main import stop_blink
def init_led():
    """ initial Setup for the LED """

    global colour
    colour = Robot.LED_Pins[0]
     

def change_led(f,m):
    """ first attribute sets colour of LED second sets mode of LED"""
    
    i = Robot.LED_Pins[f]
    if m == 0:
        # LED is off
        global stop_blink
        stop_blink = 1
        GPIO.setmode(GPIO.BCM)  # use GPIO numbers (NOT pin numbers)
        GPIO.setup(i, GPIO.OUT)
        GPIO.output(i, GPIO.LOW)
        
    elif m == 1:
        # LED is on
        stop_blink = 1
        GPIO.setmode(GPIO.BCM)  # use GPIO numbers (NOT pin numbers)
        GPIO.setup(i, GPIO.OUT)
        GPIO.output(i, GPIO.HIGH)

    elif m == 2:
        # LED flashes
        global colour
        colour = Robot.LED_Pins[f]
        stop_blink = 0

def blink_led():
    """ Turns the desired LED into endless blinking mode """
    
   
    while True:
        
        
        farbe = colour
        modus = stop_blink
        if modus == 0:
            
            GPIO.setmode(GPIO.BCM)  # use GPIO numbers (NOT pin numbers)
            GPIO.setup(farbe, GPIO.OUT)
            GPIO.output(farbe, GPIO.HIGH)
            time.sleep(0.25)
            GPIO.output(farbe, GPIO.LOW)
            time.sleep(0.25)
        elif modus == 1:
            
            GPIO.setmode(GPIO.BCM)  # use GPIO numbers (NOT pin numbers)
            GPIO.setup(farbe, GPIO.OUT)
            GPIO.output(farbe, GPIO.LOW)
            time.sleep(1.5)
   
