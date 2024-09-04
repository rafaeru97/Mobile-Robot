import RPi.GPIO as GPIO
import time


class DistanceSensor:
    def __init__(self, trigger_pin, echo_pin, timeout=1):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.timeout = timeout  # Timeout in seconds

        # Setup trigger pin as output and echo pin as input
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def get_distance(self):
        # Ensure that the trigger pin is set low
        GPIO.output(self.trigger_pin, False)
        time.sleep(0.1)

        # Send a 10µs pulse to the trigger pin
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, False)

        # Wait for the echo pin to go high and record the start time
        start_time = time.time()
        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start - start_time > self.timeout:
                return None  # Return None if timeout occurs

        # Wait for the echo pin to go low and record the end time
        start_time = time.time()
        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end - start_time > self.timeout:
                return None  # Return None if timeout occurs

        # Calculate the pulse duration
        pulse_duration = pulse_end - pulse_start

        # Calculate the distance (Speed of sound is approximately 34300 cm/s)
        distance = pulse_duration * 17150

        # Round the distance to two decimal places
        distance = round(distance, 2)

        return distance

    def cleanup(self):
        GPIO.cleanup()
