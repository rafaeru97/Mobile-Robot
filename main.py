from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper, calculate_new_position, angle_to_radians
import time

def main():
    # Tworzenie obiektów enkoderów
    left_encoder = Encoder(pin_a=19, pin_b=26, wheel_diameter=0.1, ticks_per_revolution=360)
    right_encoder = Encoder(pin_a=1, pin_b=12, wheel_diameter=0.1, ticks_per_revolution=360)

    try:
        while True:
            print(f"Left Encoder Distance: {left_encoder.get_distance():.2f} meters")
            print(f"Right Encoder Distance: {right_encoder.get_distance():.2f} meters")
            time.sleep(1)


if __name__ == '__main__':
    main()
