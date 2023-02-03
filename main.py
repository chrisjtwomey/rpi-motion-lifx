#!/usr/bin/python
import RPi.GPIO as GPIO
import os
import time
import math
import statistics
import logging.config
import lifxlan
from lifxlan import LifxLAN

PIN_PIR = 16
PIN_TRIGGER = 7
PIN_ECHO = 11

# speed of ultrasonic sound in cm/s
SPEED_OF_US_SOUND = 34300
# distance of cm that means me at my desk
AT_DESK_CM = 120
TIMEOUT_SECONDS = 90
SENSOR_TIMEOUT = 5
MAX_DATAPOINTS = 5

cwd = os.path.dirname(os.path.realpath(__file__))
logging.config.fileConfig(os.path.join(cwd, "logging.dev.ini"))
log = logging.getLogger("rpi-motion-lifx")


class SensorError(Exception):
    def __init__(self, message):
        super().__init__(message)


def main():
    lan = LifxLAN()
    g = lan.get_devices_by_location("Office")

    try:
        GPIO.setmode(GPIO.BOARD)
        # setup PIR sensor
        GPIO.setup(PIN_PIR, GPIO.IN)
        # setup ultrasonic sensor
        GPIO.setup(PIN_TRIGGER, GPIO.OUT)
        GPIO.setup(PIN_ECHO, GPIO.IN)
        GPIO.output(PIN_TRIGGER, GPIO.LOW)

        log.info("waiting for sensors to settle")
        time.sleep(2)

        detection_time = time.time()
        while True:
            nowtime = time.time()
            secs_since_detection = math.ceil(nowtime - detection_time)

            try:
                if is_motion_detected():
                    detection_time = nowtime
                    log.info("PIR sensor presence detected")
                    if secs_since_detection >= TIMEOUT_SECONDS:
                        g.set_power(True)
                else:
                    if secs_since_detection >= TIMEOUT_SECONDS:
                        log.info("PIR sensor presence timeout")
                        if at_desk():
                            log.info("ultrasonic sensor presence detected")
                            log.debug("resetting timeout")
                            continue

                        g.set_power(False)
            except lifxlan.errors.WorkflowException as we:
                log.warning(we)
            except Exception as e:
                raise e
            finally:
                time.sleep(1)
    finally:
        GPIO.cleanup()


def is_motion_detected():
    return GPIO.input(PIN_PIR)


def at_desk():
    distance_datapoints = []

    for _ in range(MAX_DATAPOINTS):
        try:
            distance_cm = us_get_distance_cm()
            distance_datapoints.append(distance_cm)
        except SensorError as se:
            log.warning(se)
        except Exception as e:
            raise e

    avg = round(sum(distance_datapoints) / len(distance_datapoints), 2)
    median = round(statistics.median(distance_datapoints), 2)

    log.debug(f"Current: {distance_cm}cm")
    log.debug(f"Average: {avg}cm")
    log.debug(f"Median: {median}cm")

    return median < AT_DESK_CM


def us_get_distance_cm():
    GPIO.output(PIN_TRIGGER, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(PIN_TRIGGER, GPIO.LOW)

    starttime = time.time()
    while GPIO.input(PIN_ECHO) == 0:
        nowtime = time.time()
        secs_since_start = math.ceil(nowtime - starttime)
        if secs_since_start > SENSOR_TIMEOUT:
            raise SensorError("Ultrasonic sensor GPIO timeout")

        pulse_start_time = nowtime

    starttime = time.time()
    while GPIO.input(PIN_ECHO) == 1:
        nowtime = time.time()
        secs_since_start = math.ceil(nowtime - starttime)
        if secs_since_start > SENSOR_TIMEOUT:
            raise SensorError("Ultrasonic sensor GPIO timeout")

        pulse_end_time = nowtime

    pulse_duration = pulse_end_time - pulse_start_time
    distance = round(pulse_duration * (SPEED_OF_US_SOUND / 2), 2)

    return distance


if __name__ == "__main__":
    main()
