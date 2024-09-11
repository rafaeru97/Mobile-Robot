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

    def reset(self):
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

        # Encoders
        self.leftEncoder = None
        self.rightEncoder = None
        self.totalDistance = 0

        self.last_left_distance = 0
        self.last_right_distance = 0

        self.mapper = None

        self.status = "Initializing..."

    def setMapper(self, mapper):
        self.mapper = mapper

    def getStatus(self):
        return self.status

    def setEncoders(self, leftEncoder, rightEncoder):
        self.leftEncoder = leftEncoder
        self.rightEncoder = rightEncoder
        self.leftEncoder.reset_position()
        self.rightEncoder.reset_position()

    def resetEncoders(self):
        self.leftEncoder.reset_position()
        self.rightEncoder.reset_position()

    def getEncoderDistance(self):
        return (self.leftEncoder.get_distance() + self.rightEncoder.get_distance()) / 2

    def forward(self):
        """Ruszanie do przodu z określoną prędkością"""
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        # self.resetEncoders()
        self.status = "Forward"

    def backward(self):
        """Ruszanie w tył z określoną prędkością"""
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        # self.resetEncoders()
        self.status = "Backward"

    def turn_left(self, speed):
        """Obracanie w lewo (lewy silnik w tył, prawy do przodu)"""
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

        # Oblicz dystans od ostatniego pomiaru
        left_distance = abs(self.leftEncoder.get_distance() - self.last_left_distance)
        right_distance = abs(self.rightEncoder.get_distance() - self.last_right_distance)

        # Zaktualizuj ostatnie wartości dystansu
        self.last_left_distance = self.leftEncoder.get_distance()
        self.last_right_distance = self.rightEncoder.get_distance()

        # Calculate error
        error = left_distance - right_distance

        # Compute PID output
        correction = self.pid.compute(error)

        # Adjust speed based on correction
        left_speed = speed - correction
        right_speed = speed + correction

        self.pwm_a.ChangeDutyCycle(left_speed)
        self.pwm_b.ChangeDutyCycle(right_speed)
        self.status = "Turn Left"

    def turn_right(self, speed):
        """Obracanie w prawo (lewy silnik do przodu, prawy w tył)"""
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

        # Oblicz dystans od ostatniego pomiaru
        left_distance = abs(self.leftEncoder.get_distance() - self.last_left_distance)
        right_distance = abs(self.rightEncoder.get_distance() - self.last_right_distance)

        # Zaktualizuj ostatnie wartości dystansu
        self.last_left_distance = self.leftEncoder.get_distance()
        self.last_right_distance = self.rightEncoder.get_distance()

        # Calculate error
        error = left_distance - right_distance

        # Compute PID output
        correction = self.pid.compute(error)

        # Adjust speed based on correction
        left_speed = speed - correction
        right_speed = speed + correction

        self.pwm_a.ChangeDutyCycle(left_speed)
        self.pwm_b.ChangeDutyCycle(right_speed)
        self.status = "Turn Right"

    def stop(self):
        """Zatrzymywanie obu silników"""
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        self.status = "Stop"

    def drive(self, speed):
        if speed == 0:
            return

        if speed > 0:
            self.forward()
        else:
            self.backward()

        speed = abs(speed)

        # Oblicz dystans od ostatniego pomiaru
        left_distance = abs(self.leftEncoder.get_distance() - self.last_left_distance)
        right_distance = abs(self.rightEncoder.get_distance() - self.last_right_distance)

        # Zaktualizuj ostatnie wartości dystansu
        self.last_left_distance = self.leftEncoder.get_distance()
        self.last_right_distance = self.rightEncoder.get_distance()

        # Calculate error
        error = left_distance - right_distance

        # Compute PID output
        correction = self.pid.compute(error)

        # Adjust speed based on correction
        left_speed = speed - correction
        right_speed = speed + correction

        # Ensure speed is within 0 to 100 range
        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))

        self.pwm_a.ChangeDutyCycle(left_speed)
        self.pwm_b.ChangeDutyCycle(right_speed)

    def forward_with_encoders(self, target_distance, base_speed=50, timeout=30):
        # Rozpocznij jazdę do przodu
        self.drive(base_speed)

        start_time = time.time()
        start_left_distance = self.last_left_distance
        start_right_distance = self.last_right_distance
        try:
            while True:
                # Steruj robotem
                self.drive(base_speed)

                left_distance = abs(self.leftEncoder.get_distance() - start_left_distance)
                right_distance = abs(self.rightEncoder.get_distance() - start_right_distance)

                # Sprawdź, czy robot osiągnął docelowy dystans
                if (left_distance + right_distance) / 2 >= target_distance:
                    self.mapper.update_position()
                    break

                elapsed_time = time.time() - start_time
                if elapsed_time > timeout:
                    self.mapper.update_position()
                    break

                time.sleep(0.02)

        finally:
            # Zatrzymaj robota
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
                    break

                elapsed_time = time.time() - start_time
                if elapsed_time > timeout:
                    self.mapper.update_position(-(left_distance + right_distance) / 2)
                    break

                time.sleep(0.1)

        finally:
            self.stop()

    def rotate_to_angle(self, gyro, target_angle, speed=48):
        """
        Obrót robota o określony kąt za pomocą żyroskopu, decydując najkrótszą drogę do kąt docelowego.

        :param gyro: Obiekt klasy Gyro
        :param target_angle: Kąt docelowy w stopniach
        :param speed: Prędkość obrotu (0-100%)
        """

        try:
            while True:
                current_angle = gyro.get_angle_z() % 360
                angle_difference = (target_angle - current_angle + 180) % 360 - 180

                # Sprawdzenie, czy osiągnięto kąt docelowy
                if abs(angle_difference) < 1:
                    self.mapper.update_position()
                    break

                # Decyzja o kierunku obrotu
                if angle_difference < 0:
                    self.turn_right(speed)
                else:
                    self.turn_left(speed)

        finally:
            self.stop()

    def cleanup(self):
        """Oczyszczanie GPIO i zatrzymywanie PWM"""
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
