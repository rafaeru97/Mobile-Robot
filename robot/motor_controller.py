import RPi.GPIO as GPIO
import time
from robot.mapper import Mapper

class PID:
    def __init__(self, kp, ki, kd, dt=0.02):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.previous_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class RotatePID:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class MotorController:
    def __init__(self, en_a=13, in1=20, in2=21, en_b=12, in3=6, in4=5):
        # Inicjalizacja pinów GPIO
        self.ENA = en_a
        self.IN1 = in1
        self.IN2 = in2
        self.ENB = en_b
        self.IN3 = in3
        self.IN4 = in4

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

        # PID Controller for speed correction
        self.pid = PID(kp=500.0, ki=0.05, kd=0.1)  # Tune these values
        self.mapper = Mapper()

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

    def forward_with_encoders(self, left_encoder, right_encoder, target_distance, base_speed=50, timeout=30):
        self.forward()

        start_time = time.time()
        left_encoder.reset_position()
        right_encoder.reset_position()

        try:
            while True:
                left_distance = abs(left_encoder.get_distance())
                right_distance = abs(right_encoder.get_distance())

                # Calculate error
                error = left_distance - right_distance

                # Compute PID output
                correction = self.pid.compute(error)

                # Adjust speed based on correction
                left_speed = base_speed - correction
                right_speed = base_speed + correction

                # Ensure speed is within 0 to 100 range
                left_speed = max(0, min(100, left_speed))
                right_speed = max(0, min(100, right_speed))

                self.pwm_a.ChangeDutyCycle(left_speed)
                self.pwm_b.ChangeDutyCycle(right_speed)

                if (left_distance + right_distance) / 2 >= target_distance:
                    self.mapper.update_position(target_distance)
                    print(f"Target distance {target_distance} meters reached.")
                    break

                elapsed_time = time.time() - start_time
                if elapsed_time > timeout:
                    self.mapper.update_position((left_distance + right_distance) / 2)
                    print("Timeout reached before target distance was achieved.")
                    break

                time.sleep(0.02)

        finally:
            self.stop()

    def backward_with_encoders(self, left_encoder, right_encoder, target_distance, base_speed=50, timeout=30):
        self.backward()

        start_time = time.time()
        left_encoder.reset_position()
        right_encoder.reset_position()

        try:
            while True:
                left_distance = abs(left_encoder.get_distance())
                right_distance = abs(right_encoder.get_distance())

                # Calculate error
                error = left_distance - right_distance

                # Compute PID output
                correction = self.pid.compute(error)

                # Adjust speed based on correction
                left_speed = base_speed - correction
                right_speed = base_speed + correction

                # Ensure speed is within 0 to 100 range
                left_speed = max(0, min(100, left_speed))
                right_speed = max(0, min(100, right_speed))

                self.pwm_a.ChangeDutyCycle(left_speed)
                self.pwm_b.ChangeDutyCycle(right_speed)

                # Ensure the distance is measured in reverse
                if (left_distance + right_distance) / 2 >= target_distance:
                    self.mapper.update_position(-target_distance)
                    print(f"Target distance {target_distance} meters reached.")
                    break

                elapsed_time = time.time() - start_time
                if elapsed_time > timeout:
                    self.mapper.update_position(-(left_distance + right_distance) / 2)
                    print("Timeout reached before target distance was achieved.")
                    break

                time.sleep(0.02)

        finally:
            self.stop()

    def rotate_to_angle(self, gyro, target_angle, direction='left', timeout=30):
        """
        Obrót robota o określony kąt za pomocą żyroskopu z regulacją PID.

        :param gyro: Obiekt klasy Gyro
        :param target_angle: Kąt docelowy w stopniach
        :param direction: Kierunek obrotu ('left' lub 'right')
        :param timeout: Maksymalny czas obrotu w sekundach
        """
        start_time = time.time()
        gyro.reset_angle()

        # Ustawienia PID
        kp = 1.0  # Współczynnik proporcjonalny
        ki = 0.0  # Współczynnik całkujący
        kd = 0.1  # Współczynnik różniczkujący
        pid = RotatePID(kp, ki, kd, target_angle)

        if direction == 'left':
            self.turn_left(0)  # Ustawienie prędkości na 0 na początku
        elif direction == 'right':
            self.turn_right(0)  # Ustawienie prędkości na 0 na początku
        else:
            raise ValueError("Invalid direction. Use 'left' or 'right'.")

        try:
            while True:
                current_angle = gyro.get_angle_z()
                dt = time.time() - start_time
                if dt > timeout:
                    print("Timeout reached before target angle was achieved.")
                    break

                # Obliczaj prędkość obrotu na podstawie PID
                control = pid.compute(current_angle, dt)
                control = max(35, min(100, control))
                print(f"control: {control}")

                if direction == 'left':
                    self.turn_left(control)
                elif direction == 'right':
                    self.turn_right(control)

                # Sprawdź, czy osiągnięto kąt docelowy
                if abs(current_angle - target_angle) < 1:  # Tolerancja 1 stopień
                    self.mapper.update_orientation(target_angle)
                    print(f"Target angle {target_angle} degrees reached [CA: {current_angle}].")
                    break

                time.sleep(0.02)  # Krótkie opóźnienie między odczytami

        finally:
            self.stop()

    def cleanup(self):
        """Oczyszczanie GPIO i zatrzymywanie PWM"""
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
