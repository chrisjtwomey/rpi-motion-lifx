#!/usr/bin/python
import RPi.GPIO as GPIO
import os
import time
import math
import logging.config
import lifxlan
from lifxlan import LifxLAN

# speed of ultrasonic sound in cm/s
US_SPEED_OF_SOUND = 34300

# timeout for PIR motion sensor
PIR_TIMEOUT_SECONDS = 90
# timeout for ultrasonic sensor
US_TIMEOUT_SECONDS = 10
# max distance in cm for ultrasonic presence
US_PRESENCE_MAX_CM = 120
# name of location for LIFX device group
LIFX_LOCATION_NAME = "Office"

# interval in seconds to poll sensors
POLL_INTERVAL = 1
# interval in seconds to wait for sensors to init
INIT_INTERVAL = 2

# pin number for HC-SR501 output
PIN_PIR = 16
# pin number for HC-SR04 trigger
PIN_TRIGGER = 7
# pin number for HC-SR04 echo
PIN_ECHO = 11


cwd = os.path.dirname(os.path.realpath(__file__))
logging.config.fileConfig(os.path.join(cwd, "logging.dev.ini"))
log = logging.getLogger("rpi-motion-lifx")


class SensorError(Exception):
    def __init__(self, message):
        super().__init__(message)


def main():
    log.info(f"PIR timeout: {PIR_TIMEOUT_SECONDS}s")
    log.info(f"US timeout: {US_TIMEOUT_SECONDS}s")
    log.info(f"Max US distance: {US_PRESENCE_MAX_CM}cm")
    log.info(f"LIFX group: {LIFX_LOCATION_NAME}")

    lifx = LifxLAN().get_devices_by_location(LIFX_LOCATION_NAME)

    try:
        GPIO.setmode(GPIO.BOARD)
        # setup PIR sensor
        GPIO.setup(PIN_PIR, GPIO.IN)
        # setup ultrasonic sensor
        GPIO.setup(PIN_TRIGGER, GPIO.OUT)
        GPIO.setup(PIN_ECHO, GPIO.IN)
        GPIO.output(PIN_TRIGGER, GPIO.LOW)

        log.info("waiting for sensors to settle")
        time.sleep(INIT_INTERVAL)

        presence_timeout = False
        pir_presence_time = us_presence_time = time.time()
        while True:
            nowtime = time.time()

            try:
                if is_motion_detected():
                    presence_timeout = False
                    # reset presence times for sensors
                    pir_presence_time = us_presence_time = nowtime
                    log.info("presence detected: PIR")

                    lifx.set_power(True)
                    continue

                secs_since_pir_presence = math.ceil(nowtime - pir_presence_time)
                if secs_since_pir_presence < PIR_TIMEOUT_SECONDS:
                    # don't start polling ultrasonic sensor until PIR timeout
                    continue

                if presence_timeout:
                    # don't continue polling ultrasonic sensor after US timeout
                    continue

                if is_ultrasonic_detected():
                    presence_timeout = False
                    # reset presence times for sensors
                    us_presence_time = pir_presence_time = nowtime
                    log.info("presence detected: US")

                secs_since_us_presence = math.ceil(nowtime - us_presence_time)
                if secs_since_us_presence < PIR_TIMEOUT_SECONDS + US_TIMEOUT_SECONDS:
                    # continue polling ultrasonic sensor until timeout
                    continue

                presence_timeout = True
                log.info("presence timeout")
                lifx.set_power(False)
            except lifxlan.errors.WorkflowException as we:
                log.warning(we)
            except Exception as e:
                raise e
            finally:
                time.sleep(POLL_INTERVAL)
    finally:
        GPIO.cleanup()


# if PIN_PIR is high, motion is detected
def is_motion_detected():
    return GPIO.input(PIN_PIR)


def is_ultrasonic_detected():
    distance_cm = 0
    try:
        distance_cm = us_get_distance_cm()
    except SensorError as se:
        log.warning(se)
    except Exception as e:
        raise e

    log.debug(f"US distance: {distance_cm}cm")

    return distance_cm < US_PRESENCE_MAX_CM


# gets the distance from ultrasonic sensor in cm
def us_get_distance_cm(timeout=1):
    GPIO.output(PIN_TRIGGER, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(PIN_TRIGGER, GPIO.LOW)

    starttime = time.time()
    while GPIO.input(PIN_ECHO) == 0:
        nowtime = time.time()
        secs_since_start = math.ceil(nowtime - starttime)
        if secs_since_start > timeout:
            raise SensorError("Ultrasonic sensor GPIO timeout")

        pulse_start_time = nowtime

    starttime = time.time()
    while GPIO.input(PIN_ECHO) == 1:
        nowtime = time.time()
        secs_since_start = math.ceil(nowtime - starttime)
        if secs_since_start > timeout:
            raise SensorError("Ultrasonic sensor GPIO timeout")

        pulse_end_time = nowtime

    pulse_duration = pulse_end_time - pulse_start_time
    distance = round(pulse_duration * (US_SPEED_OF_SOUND / 2), 2)

    return distance


if __name__ == "__main__":
    main()
