from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper, calculate_new_position, angle_to_radians
import time
import RPi.GPIO as GPIO

def main():
    motor_controller = MotorController()
    left_encoder = Encoder(pin_a=19, pin_b=26, wheel_diameter=0.1, ticks_per_revolution=960)
    right_encoder = Encoder(pin_a=1, pin_b=12, wheel_diameter=0.1, ticks_per_revolution=960)

    try:
        motor_controller.move_distance(left_encoder, right_encoder, distance=2.0, speed=50, direction='forward')
        time.sleep(2)
        motor_controller.move_distance(left_encoder, right_encoder, distance=2.0, speed=50, direction='backward')

    except KeyboardInterrupt:
        motor_controller.cleanup()

if __name__ == '__main__':
    main()
