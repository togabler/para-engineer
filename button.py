import sys

import RPi.GPIO as GPIO
import threading
import time


BUTTON_PIN = 23
T_SHUTDOWN_S = 5
EXIT_SHUTDOWN = 2


def poll_button_state():
    # Get start time for individual button press event
    t0 = time.time()

    # Poll the GPIO until it goes low
    while GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        dt_s = time.time() - t0

        # Shutdown when the key was pressed long enough, do not wait for button to be released
        if dt_s >= T_SHUTDOWN_S:
            sys.exit(EXIT_SHUTDOWN)

    # Button was finally released, evaluate the total on-time
    # Other button functions should go here, sorted from longest period to shortest


def rising_edge():
    # Start a new thread to poll the GPIO for each rising edge
    t = threading.Thread(target=poll_button_state)
    t.start()


def register_callback():
    # Configure pin as input with pull-up
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # Setup function for pin change interrupt on rising edge (dead-time 250 ms)
    GPIO.add_event_detect(BUTTON_PIN, GPIO.RISING, callback=rising_edge, bouncetime=250)

