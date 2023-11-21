import logging
import os
from sys import argv
from threading import Thread

import LED
import RPi.GPIO as GPIO
from button import EXIT_SHUTDOWN
from runtime import Runtime
from main import robotType

# main program if this file get executed
def startRobot():
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)-15s %(threadName)-15s %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s'
    )

    try:
        robot_type = robotType # import from main
    except IndexError:
        logging.exception("Need to supply robot type as command-line argument")
        raise

    app = Runtime(robot_type)
    exit_code = None

    try:
        
        LED.init_led()
        # Run the blink LED in seperate thread so we can continue in the main programm
        t2 = Thread(target=LED.blink_led)
        t2.start()
        app_initialized = True
        app.loop()
           
    except KeyboardInterrupt:
        # shutdown python program gently
        logging.exception("Stopped with KeyboardInterrupt!")
        app.program_stopped.set()
        # cleanup GPIOs (to avoid warning on next startup)
        GPIO.cleanup()
    except SystemExit as e:
        # shutdown via button
        logging.exception("Stopped via button press")
        app.program_stopped.set()
        app.lcd.print_status('Shutting down...')
        exit_code = e.code
    except Exception as e:
        logging.exception(e)
    finally:
        app.program_stopped.set()
        # cleanup GPIOs (to avoid warning on next startup)
        GPIO.cleanup()

        if exit_code == EXIT_SHUTDOWN:
            os.system("sudo shutdown now")

        # Exiting message
        logging.info("6-RUS program was terminated due to user-input or an error (Please wait ca. 5s)")
        logging.info("Please start the program again to control the robot again!")
        
if __name__ == '__main__':
    startRobot()