from time import sleep

import RPi.GPIO as GPIO


def do_steps(step_pin: int, dir_pin: int, direction=1, step_count: int = 1, delay: float = 0.02):
    """
    Makes desired steps in one motor (Pins have to be initialized already)
    `stepPin`: int  Step GPIO Pin
    `dirPin`: int  Direction GPIO Pin

    optional:
    `direction`:numeric direction > 0 -> positive, direction <= 0 -> negative
    `nrOfSteps`: int  number of steps to turn  
    `delay`: float  delay between steps in [s]
    """
    # normalize direction to 0 or 1
    direction = int(direction > 0)

    # set direction
    GPIO.output(dir_pin, direction)

    # Do steps
    for i in range(step_count):
        #print(f'Single Pin {step_pin} high')
        GPIO.output(step_pin, GPIO.HIGH)
        sleep(delay)
        #print(f'Single Pin {step_pin} low')
        GPIO.output(step_pin, GPIO.LOW)
        sleep(delay)


def do_multi_step(pins2step: list, step_pins: list, dir_pins: list, directions=None, delay: float = 0.02):
    """
    Makes one (or zero) step(s) for each motor simultaniously. All list have to be the same length
    (Pins have to be initialized already)

    `pins2Step`: list of boolean (0 or 1)   1 := do a step   0 := do no step
    `stepPins`: list of int   corresponding GPIO Pin numbers to make a step on
    `dirPins`: list of int    corresponding Direction GPIO Pin numbers
    `directions`: list of boolean (0 o 1)    corresponding direction
    `delay`: float  delay between steps in [s]
    """
    if directions is None:
        directions = [1, 1, 1, 1, 1, 1]

    # normalize direction to 0 or 1
    directions = [int(direction > 0) for direction in directions]
    
    # set all direction pins
    for i, direction in enumerate(directions):
        GPIO.output(dir_pins[i], direction)

    # set step-pins high
    for i, takeStep in enumerate(pins2step):
        if takeStep > 0:
            #print(f'Multi Pin {i} high')
            GPIO.output(step_pins[i], GPIO.HIGH)
    sleep(delay)

    # set step-pins low again
    for i, takeStep in enumerate(pins2step):
        if takeStep > 0:
            #print(f'Multi Pin {i} low')
            GPIO.output(step_pins[i], GPIO.LOW)
    sleep(delay)
