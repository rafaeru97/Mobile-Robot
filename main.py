from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper, calculate_new_position, angle_to_radians


def main():
    motor_controller = MotorController(pin1=17, pin2=18, pwm_pin=22)
    encoder = Encoder(pin_a=23, pin_b=24)
    gyro = Gyro()
    distance_sensor = DistanceSensor(trigger_pin=27, echo_pin=22)
    mapper = Mapper(map_size=(100, 100))

    while True:
        # Example loop to move robot, read sensors and update map
        motor_controller.set_speed(50)
        motor_controller.set_direction('forward')

        # Simulate a movement for example purposes
        encoder.read()
        gyro.read_rotation()
        distance = distance_sensor.get_distance()

        # Example position and angle updates
        position = encoder.get_position()
        angle = gyro.read_rotation()
        new_position = calculate_new_position(position, distance, angle_to_radians(angle))
        mapper.update_position(new_position[0], new_position[1])
        mapper.update_map(distance, angle)

        # Visualization or saving the map could be added here


if __name__ == '__main__':
    main()
