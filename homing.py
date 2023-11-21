from Quattro import Quattro
import RPi.GPIO as GPIO
import logging
import stepper
from Robot import Robot
from time import sleep


def set_steppermode(stepper_mode):
    """ This function sets the microstep pins to the desired resolution for homing """

    mode = (21,20,16)
    resolution = {
            1: (0, 0, 0),
            1 / 2: (1, 0, 0),
            1 / 4: (0, 1, 0),
            1 / 8: (1, 1, 0),
            1 / 16: (0, 0, 1),
            1 / 32: (1, 0, 1)
        }

    GPIO.setmode(GPIO.BCM)  # use GPIO numbers (NOT pin numbers)
    GPIO.setup(mode, GPIO.OUT)  # set M-pins (M0,M1,M2) as outputpins
    GPIO.output(mode, resolution[stepper_mode])  # set Mode-Pins to desired values


def homing_fast_new(steppermode,dof):
    """ This function handles the automatic calibration for the Robot.
        For a time efficient process it is run with 1/2 step mode followed by 1/32 step mode for precise positioning"""

    print('in homing')
    go_step = [1,1,1,1,1,1]
    test =[0,0,0,0,0,0]
    direction = [1,1,1,1,1,1] 
    
    set_steppermode(steppermode)

    # Check if ligthbarrier is blocked
    while go_step[:dof] != test[:dof]:
        
        for j in range(0,dof,1):
            sensor=GPIO.input(Robot.Lightbarrier_Pins[j])
            
            if sensor == 1:
                go_step[j] = 1 
            elif sensor == 0:
                go_step[j] = 0

        # rotation compensation for 6RUS    
        if dof == 6:
            direction = [1,-1,1,-1,1,-1]

        # do steps on every arm where lightbarrier is not blocked
        stepper.do_multi_step(go_step[:dof],Robot.STEP_PINS[:dof],Robot.DIR_PINS[:dof],direction[:dof])

    go_step= [1,1,1,1,1,1]
    direction = [-1,-1,-1,-1,-1,-1]

    # compensate for positioning of the motors
    if dof == 6:
        direction = [-1,1,-1,1,-1,1]

    for i in range(0,8,1):
        if steppermode == 1/32:
            # Dont move out of Position while in precise positioning
            break
        else:
            # Move out of Top Position for precise positioning
            stepper.do_multi_step(go_step[:dof],Robot.STEP_PINS[:dof],Robot.DIR_PINS[:dof],direction[:dof])
            
    set_steppermode(1 / 32)


def move_home(dof):
    """ Moves the Robot to a ready Position """

    set_steppermode(1/4)
    go_step= [1,1,1,1,1,1]
    direction = [-1,-1,-1,-1,-1,-1]

    # compensate for positioning of the motors
    if dof == 6:
        direction = [-1,1,-1,1,-1,1]
    
    for i in range(0,200,1):
       
       stepper.do_multi_step(go_step[:dof],Robot.STEP_PINS[:dof],Robot.DIR_PINS[:dof],direction[:dof])
            
    set_steppermode(1 / 32)
    