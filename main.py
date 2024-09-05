from robot import MotorController, Encoder, Gyro, DistanceSensor
import time
import RPi.GPIO as GPIO
import curses

GPIO.setmode(GPIO.BCM)

def print_gui(speed, distance, orientation):
    print("\033[H\033[J", end="")  # Czyści ekran terminala
    print("****************************")
    print("*  Robot Status              *")
    print("****************************")
    print(f"Speed:        {speed:.2f} units")
    print(f"Distance:     {distance:.2f} cm")
    print(f"Orientation:  {orientation:.2f} degrees")
    print("****************************")
    print("Press Ctrl+C to exit.")

def main(stdscr):
    # Włącz tryb nieblokujący
    stdscr.nodelay(1)
    stdscr.timeout(100)  # Czeka na 100 ms na wejście

    left_encoder = Encoder(pin_a=19, pin_b=26, wheel_diameter=0.08, ticks_per_revolution=960)
    right_encoder = Encoder(pin_a=16, pin_b=1, wheel_diameter=0.08, ticks_per_revolution=960)
    motor_controller = MotorController()
    motor_controller.setEncoders(left_encoder, right_encoder)
    gyro = Gyro()
    sensor = DistanceSensor(trigger_pin=23, echo_pin=24)

    speed = 0
    rotate = 0

    while True:
        try:
            key = stdscr.getch()

            if key == curses.KEY_UP:
                speed = min(100, speed + 5)
            elif key == curses.KEY_DOWN:
                speed = max(-100, speed - 5)
            elif key == curses.KEY_LEFT:
                speed = 0
                rotate = min(30, rotate + 5)
            elif key == curses.KEY_RIGHT:
                speed = 0
                rotate = max(-30, rotate - 5)
            elif key == ord('m'):
                print('Map saved as: map.png')
                motor_controller.mapper.save_map_as_txt()
                motor_controller.mapper.save_map_as_png()
            elif key == ord('q'):
                break

            motor_controller.drive(speed)
            if rotate > 0:
                motor_controller.rotate_to_angle(gyro, target_angle=rotate, direction='left')
            elif rotate < 0:
                rotate = abs(rotate)
                motor_controller.rotate_to_angle(gyro, target_angle=rotate, direction='right')

            rotate = 0

            print_gui(speed, sensor.get_distance(), gyro.get_angle_z())
            stdscr.refresh()
            time.sleep(0.1)  # Spowolnienie pętli

        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    curses.wrapper(main)
