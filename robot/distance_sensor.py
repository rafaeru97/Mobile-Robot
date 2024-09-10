import RPi.GPIO as GPIO
import time
import threading
import logging

class DistanceSensor:
    def __init__(self, trigger_pin, echo_pin, timeout=1, sensor_offset=16):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.timeout = timeout  # Timeout in seconds
        self.sensor_offset = sensor_offset  # sensor distance from center
        self.status = True

        # Setup trigger pin as output and echo pin as input
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

        # Lock for thread-safe access to GPIO
        self.lock = threading.Lock()

    def get_distance(self):
        with self.lock:  # Ensure thread-safe access to GPIO
            try:
                if not self.status:
                    return None

                # Ensure that the trigger pin is set low
                GPIO.output(self.trigger_pin, False)
                time.sleep(0.1)

                # Send a 10µs pulse to the trigger pin
                GPIO.output(self.trigger_pin, True)
                time.sleep(0.00001)
                GPIO.output(self.trigger_pin, False)

                # Initialize variables
                pulse_start = None
                pulse_end = None

                # Wait for the echo pin to go high and record the start time
                start_time = time.time()
                while GPIO.input(self.echo_pin) == 0:
                    pulse_start = time.time()
                    if pulse_start - start_time > self.timeout:
                        logging.error("Timeout while waiting for echo pin to go high.")
                        return None  # Return None if timeout occurs

                # Wait for the echo pin to go low and record the end time
                start_time = time.time()
                while GPIO.input(self.echo_pin) == 1:
                    pulse_end = time.time()
                    if pulse_end - start_time > self.timeout:
                        logging.error("Timeout while waiting for echo pin to go low.")
                        return None  # Return None if timeout occurs

                # Check if pulse_start and pulse_end are set
                if pulse_start is None or pulse_end is None:
                    logging.error("Pulse start or end time not set.")
                    return None

                # Calculate the pulse duration
                pulse_duration = pulse_end - pulse_start

                # Calculate the distance (Speed of sound is approximately 34300 cm/s)
                distance = pulse_duration * 17150

                # Round the distance to two decimal places
                distance = round(distance, 2)

                return distance + self.sensor_offset

            except Exception as e:
                logging.error(f"Error in get_distance: {e}")
                return None

    def cleanup(self):
        GPIO.cleanup()
