from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper, calculate_new_position, angle_to_radians
import time

def main():
    motor_controller = MotorController()
    gyro = Gyro()

    try:
        motor_controller.rotate_to_angle(gyro, 90, speed=50, direction='right')
        time.sleep(1)
        motor_controller.rotate_to_angle(gyro, 90, speed=50, direction='left')

    except KeyboardInterrupt:
        pass

    finally:
        motor_controller.cleanup()


if __name__ == '__main__':
    main()
