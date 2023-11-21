import abc
import logging
import math as m
import time

import RPi.GPIO as GPIO
import numpy as np

import stepper
from slerp import slerp_pose, angle_to_turn


class WorkspaceViolation(ValueError):
    pass


class Robot(metaclass=abc.ABCMeta):
    DIR_PINS = [13, 5, 9, 22, 17, 3]
    STEP_PINS = [6, 11, 10, 27, 4, 2]
    Lightbarrier_Pins = [14, 15, 23, 24, 25, 8]
    LED_Pins = [18, 19 ,12]

    def __init__(self, dof, stepper_mode, steps_per_rev, step_delay, rot_comp):
        self.dof = dof

        # Robot GPIO-pins:
        self.M0 = 21
        self.M1 = 20
        self.M2 = 16
        self.dirPins = self.DIR_PINS[:dof]
        self.stepPins = self.STEP_PINS[:dof]
        self.lightbarrierpins = self.Lightbarrier_Pins[:dof]
        self.ledpins = self.LED_Pins[:3]
        self.enablePin = 0
        self.rotation_compensation = rot_comp

        self.currPose = [0.0] * dof  # current pose of the robot: [x, y, z, alpha, beta, gamma]
        self.currSteps = [0] * dof  # current motorangles as steps #TODO: this varriable is not updated yet
        self.homePose = [0.0] * dof # homing pose of robot: [x, y, z, alpha, beta, gamma]

        self.stepAngle = stepper_mode * 2 * m.pi / steps_per_rev  # angle corresponding to one step
        self.stepperMode = stepper_mode  # set mode to class variable
        self.stepDelay = stepper_mode * step_delay  # calculate time between steps
        self.init_gpio()  # initialise needed GPIO-pins

    def init_gpio(self):
        """This initialises all GPIO pins of the Raspberry Pi that are needed.
        The pins are hardcoded and defined in the documentation! If they have to be
        changed, edit the corresponting variables in this class"""
        GPIO.setmode(GPIO.BCM)  # use GPIO numbers (NOT pin numbers)

        # set up microstep pins
        mode = (self.M0, self.M1, self.M2)  # microstep resolution GPIO pins
        GPIO.setup(mode, GPIO.OUT)  # set M-pins (M0,M1,M2) as outputpins

        # dictionary for microstep mode
        resolution = {
            1: (0, 0, 0),
            1 / 2: (1, 0, 0),
            1 / 4: (0, 1, 0),
            1 / 8: (1, 1, 0),
            1 / 16: (0, 0, 1),
            1 / 32: (1, 0, 1)
        }

        GPIO.output(mode, resolution[self.stepperMode])  # set Mode-Pins to desired values

        # init LED-Pins
        for i in self.ledpins:
            GPIO.setup(i,GPIO.OUT)
            GPIO.output(i, GPIO.LOW)
            
        # init Lightbarrier-Pins
        for i in self.lightbarrierpins:
            GPIO.setup(i, GPIO.IN)

        # init Step-Pins as output-pins
        for i in self.stepPins:
            GPIO.setup(i, GPIO.OUT)
            GPIO.output(i, GPIO.LOW)

        # init Direction-Pins as output-pins
        for i in self.dirPins:
            GPIO.setup(i, GPIO.OUT)
            GPIO.output(i, GPIO.LOW)

        # set !ENABLE-Pin Low
        GPIO.setup(self.enablePin, GPIO.OUT)
        # Start disabled to avoid noise
        GPIO.output(self.enablePin, GPIO.HIGH)

    def enable_steppers(self):
        GPIO.output(self.enablePin, GPIO.LOW)

    def disable_steppers(self):
        GPIO.output(self.enablePin, GPIO.HIGH)

    def angles2steps(self, angles: list):
        """converts list of angles [rad] to list of steps"""
        steps = [round(x / self.stepAngle) for x in angles]
        return steps

    def homing(self, method: str) -> None:
        """Homing of the Robot
        `method`:str  chooses the method of homing

        Methods:
        ------
        '90': All arms get positioned automatically with the lightbarriers
        """
        if method == '90':
            # Homing with Homeposition: Angles -> (90°,90°,90°,90°,90°,90°)
            angles = [m.pi / 2] * self.dof
            homing_pose = self.forward_kinematic(angles)  # calculate the position via forward kinematics
            logging.info(f'Robot is now at homing pose: {homing_pose}')
            self.currPose = homing_pose
            self.homePose = homing_pose           
            self.currSteps = self.angles2steps(angles)
        else:
            raise ValueError('Chosen homing-method is not defined!')

    def mov_steps(self, step_list, new_pose: list):
        """ This funktion moves every motor x steps, where x are the number of steps to take
        `stepList` is a np-array or list with 6 Values for the steps to take
        `newPose`:list is the pose after the movement was done
        """
        new_pose = new_pose[:self.dof]
        step_list = step_list[:self.dof]

        # movVec = np.array([2, -5, 1, -10, 0, 0])
        mov_vec = np.array(step_list)  # convert to np.array for vector calculations

        # compensate for motor placement (switch direction every second motor)
        mov_vec = np.multiply(mov_vec, self.rotation_compensation)

        step_count = np.zeros(self.dof, dtype=int)  # step counter for calculating on wich loop to move

        max_steps = int(max(abs(mov_vec)))  # maximum steps to move
        step_after_inc = max_steps / (abs(mov_vec) + 1)  # after which increments to take one step

        # determine direction from sign of vector-element
        directions = [0] * self.dof  # saves in which directions the motors should turn
        for i, _ in enumerate(directions):
            if mov_vec[i] < 0:
                directions[i] = 0
            else:
                directions[i] = 1

        for i in range(max_steps):  # loop with step for highest amount of steps
            step_motors = [0] * self.dof  # reset

            for n, incNr in enumerate(step_after_inc):  # loop trough all motor step-values
                # n in here is the number of the targetmotor. Starting from 0
                if np.isinf(incNr):  # if number is infinite skip loop (happens if steps to move are 0)
                    continue

                c = round(((step_count[n] + 1) * incNr))

                if c == i or incNr < 1:  # test if a step should be executed
                    step_count[n] += 1  # Adding steps for calculating the next step
                    step_motors[n] = 1  # Add that this motor should

            # Execute steps for motors, if they should step
            stepper.do_multi_step(step_motors, self.stepPins, self.dirPins, directions, delay=self.stepDelay)

        # Update current pose and current steps
        self.currSteps = list(np.array(self.currSteps) + np.array(step_list))
        self.currPose = new_pose

    # MOVING
    def mov(self, pose: list):
        """Move to new position/pose with Point-to-Point (PTP) interpolation.
        This is a synchronous PTP implementation"""
        pose = pose[:self.dof]

        new_angles = self.inv_kinematic(pose)  # get new angles
        #logging.info(f'New joint angles: {new_angles}')
        new_steps = self.angles2steps(new_angles)  # calculate steps of new position

        # create list of steps to move
        #logging.info(f'New steps: {new_steps}')
        #logging.info(f'Current steps: {self.currSteps}')
        steps_to_move = np.array(new_steps) - np.array(self.currSteps)
        
        # move motors corresponding to stepsToMove-list
        #logging.info(f'Moving to {pose}')
        self.mov_steps(steps_to_move, pose)

    def mov_lin(self, pose: list, pos_res: float = 10, ang_res: float = 3, vel: float = None) -> None:
        """
        Move to new position with linear interpolation
        `pose`: list with values of the pose to move to
        `posRes`: how many interpolating points should be used in [steps in cm]
        `angRes`: how many interpolating points should be used in [steps in (10*deg)]
        `vel`: how fast the robot should move [cm/s] (default is as fast as possible)
        """
        pose = pose[:self.dof]

        # Calculate distance to move
        x_dir = pose[0] - self.currPose[0]
        y_dir = pose[1] - self.currPose[1]
        z_dir = pose[2] - self.currPose[2]

        distance = m.sqrt(x_dir ** 2 + y_dir ** 2 + z_dir ** 2)  # distance to move [mm]
        steps_pos = distance * pos_res / 10  # Number of steps to move (calculated by distance)

        # Calculate angle to move
        angle_to_turn_val = angle_to_turn(self.currPose, pose)
        steps_rot = m.degrees(angle_to_turn_val) * ang_res / 10  # Number of steps to move (calculated by angle)

        # take the maximum steps needed to match resolution
        nr_of_steps = m.ceil(max([steps_pos, steps_rot]))

        if nr_of_steps <= 0:
            return  # return if poses are already identical

        # check if velocity was given
        if vel is not None:
            if vel > 0:
                # Calculate the timing for velocity management
                t_ges = (distance / 10) / vel  # calculate duration of whole movement
                dt_ideal = t_ges / nr_of_steps  # calculate time it should take to execute one loop iteration
            else:
                logging.warning('Given velocity is lower than 0 or 0! Using default!')

        poses = slerp_pose(self.currPose, pose, nr_of_steps + 1)  # calculate poses in between
        t_st = time.time()  # check the time

        for i, poseBetw in enumerate(poses):
            try:
                self.mov(poseBetw)  # go to next pose
            except WorkspaceViolation:
                break
            self.currPose = poseBetw  # update Pose

            # Velocity management
            if vel is not None:  # if velocity is given make calculations for velocity-management
                t_ideal = t_st + (i + 1) * dt_ideal  # calculate ideal time (to compare to)
                t_curr = time.time()  # get current time
                dt = t_ideal - t_curr  # copmarison between ideal and real timing

                if dt > self.stepDelay:  # if ideal timing is in front of the real time
                    time.sleep(dt)  # sleep until timings match again

                elif dt < -2 * self.stepDelay:  # if real time is too much behind
                    # TODO: Maybe turn on LED or something (just printing on console in this case)
                    logging.warning('Can not keep velocity!')

    @abc.abstractmethod
    def inv_kinematic(self, pose: list):
        pass

    @abc.abstractmethod
    def forward_kinematic(self, angles):
        pass

    @abc.abstractmethod
    def change_robot_dimensions(self, *args):
        pass
