import RPi.GPIO as GPIO

class MotorController:
    def __init__(self, en_a=13, in1=20, in2=21, en_b=12, in3=6, in4=5):
        # Inicjalizacja pinów GPIO
        self.ENA = en_a
        self.IN1 = in1
        self.IN2 = in2
        self.ENB = en_b
        self.IN3 = in3
        self.IN4 = in4

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)

        # Inicjalizacja PWM
        self.pwm_a = GPIO.PWM(self.ENA, 1000)  # PWM dla silnika A (1000 Hz)
        self.pwm_b = GPIO.PWM(self.ENB, 1000)  # PWM dla silnika B (1000 Hz)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def forward(self, speed):
        """Ruszanie do przodu z określoną prędkością"""
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(speed)
        self.pwm_b.ChangeDutyCycle(speed)

    def backward(self, speed):
        """Ruszanie w tył z określoną prędkością"""
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(speed)
        self.pwm_b.ChangeDutyCycle(speed)

    def turn_left(self, speed):
        """Obracanie w lewo (obracanie lewego silnika w tył, prawego do przodu)"""
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(speed)
        self.pwm_b.ChangeDutyCycle(speed)

    def turn_right(self, speed):
        """Obracanie w prawo (obracanie prawego silnika w tył, lewego do przodu)"""
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(speed)
        self.pwm_b.ChangeDutyCycle(speed)

    def stop(self):
        """Zatrzymywanie obu silników"""
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)

    def cleanup(self):
        """Oczyszczanie GPIO i zatrzymywanie PWM"""
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()