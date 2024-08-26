from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper, calculate_new_position, angle_to_radians
import time

def main():
    controller = MotorController()

    try:
        print("Moving forward")
        controller.forward(75)  # 75% prędkości
        time.sleep(2)

        print("Turning left")
        controller.turn_left(75)
        time.sleep(2)

        print("Turning right")
        controller.turn_right(75)
        time.sleep(2)

        print("Moving backward")
        controller.backward(75)
        time.sleep(2)

        print("Stopping")
        controller.stop()
        time.sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        controller.cleanup()


if __name__ == '__main__':
    main()
