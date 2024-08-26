import RPi.GPIO as GPIO
import math

class Encoder:
    def __init__(self, pin_a, pin_b, wheel_diameter, ticks_per_revolution):
        GPIO.setmode(GPIO.BCM)  # Ustawienie trybu numeracji GPIO

        self.pin_a = pin_a
        self.pin_b = pin_b
        self.position = 0
        self.wheel_diameter = wheel_diameter  # Średnica koła w metrach
        self.ticks_per_revolution = ticks_per_revolution

        # Obliczenie liczby impulsów na metr (uwzględniając przekładnię)
        self.pulses_per_meter = self.ticks_per_revolution / (self.wheel_diameter * math.pi)

        # Konfiguracja pinów GPIO
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.last_state = GPIO.input(self.pin_a)  # Odczyt stanu pinu po konfiguracji

        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self.update_position)

    def update_position(self, channel):
        state_a = GPIO.input(self.pin_a)
        state_b = GPIO.input(self.pin_b)

        if state_a != self.last_state:
            if state_b != state_a:
                self.position += 1
            else:
                self.position -= 1

        self.last_state = state_a

    def get_position(self):
        return self.position

    def get_distance(self):
        return (self.get_position() / self.pulses_per_meter) / 2

    def reset_position(self):
        self.position = 0
