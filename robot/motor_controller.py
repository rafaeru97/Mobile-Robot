import RPi.GPIO as GPIO
import time

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
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
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

        # PID Controller for speed correction
        self.pid = PID(kp=30.0, ki=0.1, kd=0.01)  # Tune these values

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
                correction = self.pid(error)

                # Adjust speed based on correction
                left_speed = base_speed - correction
                right_speed = base_speed + correction

                print(f"L Dist: {left_distance} | R Dist: {right_distance} | Err: {error} | Corr: {correction}")

                # Ensure speed is within 0 to 100 range
                left_speed = max(0, min(100, left_speed))
                right_speed = max(0, min(100, right_speed))

                # Apply speeds
                self.pwm_a.ChangeDutyCycle(left_speed)
                self.pwm_b.ChangeDutyCycle(right_speed)

                # Check if target distance is reached
                if (left_distance + right_distance) / 2 >= target_distance:
                    print(f"Target distance {target_distance} meters reached.")
                    break

                # Check timeout
                elapsed_time = time.time() - start_time
                if elapsed_time > timeout:
                    print("Timeout reached before target distance was achieved.")
                    break

                time.sleep(0.02)

        finally:
            self.stop()

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

                print(f"L Dist: {left_distance} | R Dist: {right_distance} | Err: {error} | Corr: {correction}")

                # Ensure speed is within 0 to 100 range
                left_speed = max(0, min(100, left_speed))
                right_speed = max(0, min(100, right_speed))

                self.pwm_a.ChangeDutyCycle(left_speed)
                self.pwm_b.ChangeDutyCycle(right_speed)

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
