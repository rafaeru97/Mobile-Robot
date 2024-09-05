from robot import MotorController, Encoder, Gyro, DistanceSensor
import time
import curses

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

def showMessage(stdscr, message):
    stdscr.clear()
    stdscr.addstr(0, 0, message)
    stdscr.refresh()

def main(stdscr):
    left_encoder = Encoder(pin_a=19, pin_b=26, wheel_diameter=0.08, ticks_per_revolution=960)
    right_encoder = Encoder(pin_a=16, pin_b=1, wheel_diameter=0.08, ticks_per_revolution=960)
    motor_controller = MotorController()
    motor_controller.setEncoders(left_encoder, right_encoder)
    gyro = Gyro()
    sensor = DistanceSensor(trigger_pin=23, echo_pin=24)

    # Włącz tryb nie-blokujący w curses
    stdscr.nodelay(1)
    stdscr.timeout(100)
    last_key = None
    last_key_time = time.time()

    speed = 0
    rotate = 0

    try:
        while True:
            # showMessage(stdscr, f"Current value: {gyro.get_angle_z():.2f}")
            current_time = time.time()
            key = stdscr.getch()

            if key != -1:
                if key != last_key or (current_time - last_key_time) > 0.1:
                    if key == curses.KEY_UP:
                        speed = min(100, speed + 5)
                        showMessage(stdscr, f'Moving Forward (Speed: {str(speed)})\n')
                    elif key == curses.KEY_DOWN:
                        speed = max(-100, speed - 5)
                        showMessage(stdscr, f'Moving Backward (Speed: {str(speed)})\n')
                    elif key == curses.KEY_LEFT:
                        speed = 0
                        rotate = min(30, rotate + 5)
                        showMessage(stdscr, 'Rotating Left\n')
                    elif key == curses.KEY_RIGHT:
                        speed = 0
                        rotate = max(-30, rotate - 5)
                        showMessage(stdscr, 'Rotating Right\n')
                    elif key == ord('d'):
                        showMessage(stdscr, 'Reading distance\n')
                        distance = sensor.get_distance()
                        print(f"Distance sensor: {distance} cm")
                    elif key == ord('m'):
                        showMessage(stdscr, 'Map saved as: map.png\n')
                        motor_controller.mapper.save_map_as_txt()
                        motor_controller.mapper.save_map_as_png()
                    elif key == ord('a'):
                        stdscr.clear()
                        stdscr.refresh()
                        print(f"gyro.get_angle_z(): {gyro.get_angle_z()}")
                    elif key == ord('q'):
                        showMessage(stdscr, 'Quitting')
                        break

                    last_key = key
                    last_key_time = current_time

            motor_controller.drive(speed)

            if rotate > 0:
                motor_controller.rotate_to_angle(gyro, target_angle=rotate, direction='left')
            elif rotate < 0:
                rotate = abs(rotate)
                motor_controller.rotate_to_angle(gyro, target_angle=rotate, direction='right')

            rotate = 0
            time.sleep(0.1)

    except KeyboardInterrupt:
        stdscr.clear()
        stdscr.addstr(1, 0, 'Interrupted by user.')
        stdscr.refresh()
        time.sleep(1)  # Daj chwilę na zobaczenie komunikatu

    finally:
        motor_controller.stop()
        motor_controller.cleanup()

if __name__ == '__main__':
    curses.wrapper(main)
