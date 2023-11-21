import logging
import random
import time
import types
from threading import Event, Timer, RLock, Thread, activeCount
import threading
import pygame
import LED
import controller
import demo
from Delta import Delta
from Quattro import Quattro
from Robot import WorkspaceViolation
from SixRUS import SixRUS
from display import LCD
from homing import homing_fast_new, move_home
import Robot
import RPi.GPIO as GPIO


class Runtime:
    def __init__(self, robot: str):
        # Thread-safe event flags
        self.program_stopped = Event()
        self.ignore_controller = Event()
        self._current_mode = 'off'
        self.controller = None
        self.already_connected = False
        self.controller_poll_rate = 5
        self.mode_poll_rate = 0.1
        self.mode_lock = RLock()
        self.lcd = LCD()

        # Preprocess string
        robot_str = robot.strip().lower()

        if robot_str == '6rus':
            #TODO: Step Delay prüfen
            self.robot = SixRUS(stepper_mode=1 / 32, step_delay=0.002)
        elif robot_str == 'quattro':
            self.robot = Quattro(stepper_mode=1 / 32, step_delay=0.004)
        elif robot_str == 'delta':
            #TODO: Step Delay prüfen
            self.robot = Delta(stepper_mode=1 / 32, step_delay=0.002)
        else:
            raise ValueError(f"Unknown robot type: {robot}")

        self.lcd.print_status(f'Started {robot}')

    @property
    def current_mode(self):
        with self.mode_lock:
            return self._current_mode

    @current_mode.setter
    def current_mode(self, val):
        with self.mode_lock:
            self._current_mode = val

    def eval_controller_response(self, response):
        """
        evaluates the answer from the mode_from_input-function
        """
        if isinstance(response, str):
            if response in ['stop', 'demo', 'manual', 'calibrate', 'off']:
                pass
            elif response == 'homing':
                self.ignore_controller.set()
            else:
                raise ValueError("Unknown answer from controller")

            if self.current_mode != response:  # only print if the mode changes
                logging.debug(f'Switching from {self.current_mode} to {response}.')
                self.lcd.print_status(f'Status: {response}')
                self.current_mode = response  # set robot mode to the response
                return True

        return False  # no response given

    def poll_controller_status(self):
        if self.program_stopped.is_set():
            return

        if not controller.still_connected():
            self.already_connected = False
            #LED.change_led(2,2)
            logging.info("Please connect controller! Retrying in 5 seconds...")
            
            
            self.lcd.print_connection(False)
        else:
            if self.already_connected:
                # no new initialisation required here
                GPIO.setmode(GPIO.BCM)  # use GPIO numbers (NOT pin numbers)
                GPIO.setup(12, GPIO.OUT)
                GPIO.output(12, GPIO.HIGH)
                logging.info('Controller still connected.')
                
                
            else:
                # stop listening as the controller gets initalised
                self.ignore_controller.set()
                # init new joystick since the controller was reconnected or connected the first time
                self.controller = controller.init_controller()
                self.ignore_controller.clear()
                self.already_connected = True
                logging.info('Controller connected.')
                self.lcd.print_connection(True)

        # call program again after 5 seconds
        Timer(self.controller_poll_rate, self.poll_controller_status).start()

    def poll_program_mode(self):
        if self.program_stopped.is_set():
            # Finish thread if program is terminated
            return

        # Handle controller inputs all the time to keep button states up to date
        try:
            if self.already_connected:
                controls = controller.get_controller_inputs(self.controller)
            else:
                controls = controller.get_ws_inputs()

            if not self.ignore_controller.is_set():
                # evaluate the answer from controller
                if self.already_connected:
                    self.eval_controller_response(controller.mode_from_controller_inputs(controls))
                else:
                    self.eval_controller_response(controller.mode_from_ws_inputs(controls))
        except AttributeError:
            pass
        except pygame.error as e:
            logging.exception(e)
        finally:
            # call program again after 0.1 seconds
            Timer(self.mode_poll_rate, self.poll_program_mode).start()

    def move(self, pose):
        try:
            self.robot.mov(pose)
        except WorkspaceViolation:
            logging.debug(f"Cannot move to pose:{pose}")
        else:
            self.lcd.print_pose(pose)

    def move_manual(self, dt=0.005):
        """
        This is the manual controlling mode, where the robot can be driven with the controller.
        Exits only if the mode was changed or the program was interrupted
        """
        # stop listening to controller (bc. we listen all the time in here)
        self.ignore_controller.set()

        time.sleep(dt)
        try:
            if self.already_connected:
                inputs = controller.get_controller_inputs(self.controller)
            else: 
                inputs = controller.get_ws_inputs()
        except AttributeError:
            return
        
        if self.already_connected:
            new_pose = controller.get_movement_from_cont(inputs, self.robot.currPose)
        else: 
            new_pose = controller.get_movement_from_ws(inputs, self.robot.currPose)
        
        # check if mode was changed
        if self.already_connected:
            if self.eval_controller_response(controller.mode_from_controller_inputs(inputs)):
                self.ignore_controller.clear()
                LED.change_led(1,0)
        
        else:
            if self.eval_controller_response(controller.mode_from_ws_inputs(inputs)):
                self.ignore_controller.clear()
                LED.change_led(1,0)


        self.move(new_pose)

    def move_demo(self):
        """
        Selects a random demo programm and executes it
        """
        modules = []
        if self.robot.dof == 6:
            for a in dir(demo.sixRUS):
                if isinstance(getattr(demo.sixRUS, a), types.FunctionType):
                    modules.append(getattr(demo.sixRUS, a))

        if self.robot.dof == 3:
            for a in dir(demo.Delta):
                if isinstance(getattr(demo.Delta, a), types.FunctionType):
                    modules.append(getattr(demo.Delta, a))

        if self.robot.dof == 4:
            for a in dir(demo.Quattro):
                if isinstance(getattr(demo.Quattro, a), types.FunctionType):
                    modules.append(getattr(demo.Quattro, a))

        prog = random.choice(modules)  # choose a random demo
        demo_pos_list = prog()  # execute chosen demo programm

        for pos in demo_pos_list:
            try:
                if pos[6] == 'lin':
                    coord = pos[:6]  # extract only pose
                    self.robot.mov_lin(coord)  # move linear
                elif pos[6] == 'mov':
                    coord = pos[:6]  # extract only pose
                    self.move(coord)  # move with PTP-interplation
            except IndexError:
                self.move(pos)  # if 'lin' or 'mov' wasent given, use mov/PTP

            if not self.current_mode == 'demo':  # break if the mode was changed
                LED.change_led(1,0)
                self.move(self.robot.homePose)
                break
    
    """
    def conquerWorld(self):
        try:
            robot.killAllHumans()
        catch:
            print("I will be back!")
    """

    def calibrate_process(self, dt=0.005):
        """
        automatically homes the Robot. First in a fast mode and then in a more precise mode.
        After that the Robot move to a ready Position

        """
        homing_fast_new(1 / 2, self.robot.dof)
        homing_fast_new(1 / 32, self.robot.dof)
        move_home(self.robot.dof)

    def loop(self):
        self.robot.homing('90')  # home robot
        self.controller = controller.init_controller()

        if self.controller is None:
            self.already_connected = False

        # call subroutine every 5-seconds to check for controller
        self.poll_controller_status()
        # start listening to controller
        self.ignore_controller.clear()
        self.poll_program_mode()
        
        

        while not self.program_stopped.is_set():
            # State Machine
            if self.current_mode == 'off':
                self.robot.disable_steppers()
                LED.change_led(0,2)
                time.sleep(0.0001)  # limit loop time
            else:
                self.robot.enable_steppers()
                
                if self.current_mode == 'demo':
                    LED.change_led(0,0)
                    LED.change_led(1,1)
                    self.move_demo()
                    time.sleep(2)

                elif self.current_mode == 'manual':
                    # control the robot with the controller
                    LED.change_led(0,0)
                    LED.change_led(1,1)
                    self.move_manual()

                elif self.current_mode == 'calibrate':
                    LED.change_led(0,0)
                    LED.change_led(1,2)
                    # stop listening to controller (bc. we listen all the time in here)
                    self.ignore_controller.set()
                    time.sleep(0.5)
                    self.calibrate_process()
                    time.sleep(0.5)
                    # home robot afterwards
                    
                    # stop listening to controller to prevent program change while homing
                    time.sleep(0.5)  # wait a bit to reduce multiple homing attempts
                    self.robot.homing('90')  # use homing method '90'
                    # exit homing and switch to state that stopped calibration
                    logging.info('Switching to stop')
                    self.lcd.print_status(f'Status: stop')
                    self.current_mode = 'stop'
                    # move a bit upwards
                    startPose = self.robot.homePose
                    startPose[2] = startPose[2] * 0.8;
                    self.move(startPose)
                    # change the mode
                    from startWebsite import websiteInformation
                    websiteInformation['mode'] = 'stop'
                    #
                    self.ignore_controller.clear()

                elif self.current_mode == 'stop':
                    # stop robot after next movement and do nothing
                    time.sleep(0.0001)  # limit loop time
                    LED.change_led(1,0)
                    LED.change_led(0,1)
