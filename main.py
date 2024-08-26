from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper, calculate_new_position, angle_to_radians
import time

def main():
    gyro = Gyro()
    while True:
        angle_z = gyro_z.get_angle_z()
        print(f"Angle Z: {angle_z:.2f} degrees")
        time.sleep(0.01)  # Co 0.01 sekundy


if __name__ == '__main__':
    main()
