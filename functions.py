import math

# This file contains functions that do not require the vehicle object

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def is_tank_full():
    import RPi.GPIO as GPIO
    import time

    tank_height = 30  # in centimeters
    sensor_dist = 35  # Distance between the ultrasonic sensor and the bottom of the tank
    full_sensor_dist = sensor_dist - tank_height  # distance between the sensor and the tank when it is full

    GPIO.setmode(GPIO.BCM)

    TRIG = 2
    ECHO = 3
    i = 0

    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.setup(4, GPIO.OUT)

    GPIO.output(TRIG, False)
    print("Starting.....")
    time.sleep(2)

    while True:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO) == 1:
            pulse_stop = time.time()

        pulse_time = pulse_stop - pulse_start

        distance = pulse_time * 17150
        print(round(distance, 2));

        time.sleep(1)

        if distance < full_sensor_dist + 5:  # 5cm from the top of the tank
            print("Tank is full")
            break

    return True


def is_tank_empty():
    import RPi.GPIO as GPIO
    import time

    tank_height = 30  # in centimeters
    sensor_dist = 35  # Distance between the ultrasonic sensor and the bottom of the tank

    GPIO.setmode(GPIO.BCM)

    TRIG = 2
    ECHO = 3
    i = 0

    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.setup(4, GPIO.OUT)

    GPIO.output(TRIG, False)
    print("Starting.....")
    time.sleep(2)

    while True:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO) == 1:
            pulse_stop = time.time()

        pulse_time = pulse_stop - pulse_start

        distance = pulse_time * 17150
        print(round(distance, 2));

        time.sleep(1)

        if distance > sensor_dist - 5:  # 5cm above the bottom of the tank
            print("Tank is empty")
            break

    return True

