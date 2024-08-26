import RPi.GPIO as GPIO
import time

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

    def forward(self):
        """Ruszanie do przodu z określoną prędkością"""
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

    def backward(self):
        """Ruszanie w tył z określoną prędkością"""
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

    def turn_left(self, speed):
        """Obracanie w lewo (lewy silnik w tył, prawy do przodu)"""
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(speed)
        self.pwm_b.ChangeDutyCycle(speed)

    def turn_right(self, speed):
        """Obracanie w prawo (lewy silnik do przodu, prawy w tył)"""
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

    def rotate_to_angle(self, gyro, target_angle, speed=50, direction='left', timeout=30):
        """
        Obrót robota o określony kąt za pomocą żyroskopu z debugowaniem.
        """
        print("Starting rotation...")
        start_time = time.time()
        gyro.reset_angle()

        # Ustaw kierunek obrotu
        if direction == 'left':
            self.turn_left(speed)
        elif direction == 'right':
            self.turn_right(speed)
        else:
            raise ValueError("Invalid direction. Use 'left' or 'right'.")

        try:
            while True:
                current_angle = gyro.get_angle_z()

                print(f"Current angle: {current_angle} | Target angle: {target_angle}")

                if abs(current_angle) >= target_angle:
                    print(f"Target angle {target_angle} degrees reached.")
                    break

                elapsed_time = time.time() - start_time
                if elapsed_time > timeout:
                    print("Timeout reached before target angle was achieved.")
                    break

                time.sleep(0.02)

        finally:
            self.stop()

    def forward_with_encoders(self, left_encoder, right_encoder, target_distance, base_speed=50, timeout=30):
        """
        Ruszanie do przodu z kontrolą prędkości za pomocą enkoderów.
        """
        self.forward()

        start_time = time.time()
        left_encoder.reset_position()
        right_encoder.reset_position()

        try:
            while True:
                left_distance = abs(left_encoder.get_distance())
                right_distance = abs(right_encoder.get_distance())

                # Korekcja prędkości na podstawie różnicy odległości
                if left_distance > right_distance:
                    self.pwm_a.ChangeDutyCycle(base_speed - 5)  # Zmniejsz prędkość lewego silnika
                    self.pwm_b.ChangeDutyCycle(base_speed)
                elif right_distance > left_distance:
                    self.pwm_a.ChangeDutyCycle(base_speed)
                    self.pwm_b.ChangeDutyCycle(base_speed - 5)  # Zmniejsz prędkość prawego silnika
                else:
                    self.pwm_a.ChangeDutyCycle(base_speed)
                    self.pwm_b.ChangeDutyCycle(base_speed)

                if (left_distance + right_distance) / 2 >= target_distance:
                    print(f"Target distance {target_distance} meters reached.")
                    break

                elapsed_time = time.time() - start_time
                if elapsed_time > timeout:
                    print("Timeout reached before target distance was achieved.")
                    break

                time.sleep(0.02)

        finally:
            self.stop()

    def backward_with_encoders(self, left_encoder, right_encoder, target_distance, base_speed=50, timeout=30):
        """
        Ruszanie do tyłu z kontrolą prędkości za pomocą enkoderów.
        """
        self.backward()

        start_time = time.time()
        left_encoder.reset_position()
        right_encoder.reset_position()

        try:
            while True:
                left_distance = abs(left_encoder.get_distance())
                right_distance = abs(right_encoder.get_distance())

                # Korekcja prędkości na podstawie różnicy odległości
                if left_distance > right_distance:
                    self.pwm_a.ChangeDutyCycle(base_speed - 5)  # Zmniejsz prędkość lewego silnika
                    self.pwm_b.ChangeDutyCycle(base_speed)
                elif right_distance > left_distance:
                    self.pwm_a.ChangeDutyCycle(base_speed)
                    self.pwm_b.ChangeDutyCycle(base_speed - 5)  # Zmniejsz prędkość prawego silnika
                else:
                    self.pwm_a.ChangeDutyCycle(base_speed)
                    self.pwm_b.ChangeDutyCycle(base_speed)

                # Upewnij się, że cofanie jest mierzone w przeciwną stronę
                if (left_distance + right_distance) / 2 >= target_distance:
                    print(f"Target distance {target_distance} meters reached.")
                    break

                elapsed_time = time.time() - start_time
                if elapsed_time > timeout:
                    print("Timeout reached before target distance was achieved.")
                    break

                time.sleep(0.02)

        finally:
            self.stop()

    def cleanup(self):
        """Oczyszczanie GPIO i zatrzymywanie PWM"""
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
