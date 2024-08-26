from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper, calculate_new_position, angle_to_radians
import time

def main():
    gyro = Gyro()
    while True:
        angle_x, angle_y, angle_z = gyro.get_angles()
        print(f"Angle X: {angle_x:.2f} degrees, Angle Y: {angle_y:.2f} degrees, Angle Z: {angle_z:.2f} degrees")
        time.sleep(0.1)  # Co 0.1 sekundy


if __name__ == '__main__':
    main()
