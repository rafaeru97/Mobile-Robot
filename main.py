from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper, calculate_new_position, angle_to_radians
import time

def main():
    gyro = Gyro()
    while True:
        gx, gy, gz = gyro.read_rotation()
        print(f"Gyro X: {gx:.2f} °/s, Gyro Y: {gy:.2f} °/s, Gyro Z: {gz:.2f} °/s")
        time.sleep(0.5)


if __name__ == '__main__':
    main()
