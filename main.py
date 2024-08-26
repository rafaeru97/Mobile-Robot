from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper, calculate_new_position, angle_to_radians
import time
import RPi.GPIO as GPIO

def main():
    # Inicjalizacja kontrolera silników i enkoderów
    motor_controller = MotorController()
    left_encoder = Encoder(pin_a=19, pin_b=26, wheel_diameter=0.1, ticks_per_revolution=960)
    right_encoder = Encoder(pin_a=1, pin_b=12, wheel_diameter=0.1, ticks_per_revolution=960)

    try:
        # Ruch do przodu na odległość 2 metrów
        motor_controller.forward_with_encoders(left_encoder, right_encoder, target_distance=2.0, base_speed=50)
        time.sleep(2)  # Przerwa przed kolejnym ruchem

        # Ruch do tyłu na odległość 2 metrów
        motor_controller.backward_with_encoders(left_encoder, right_encoder, target_distance=2.0, base_speed=50)

    except KeyboardInterrupt:
        motor_controller.cleanup()

if __name__ == '__main__':
    main()
