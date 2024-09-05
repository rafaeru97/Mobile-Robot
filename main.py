from robot import MotorController, Encoder, Gyro, DistanceSensor
import time
import curses
import threading

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

def drive_thread(motor_controller, speed_event):
    while True:
        speed = speed_event.wait()
        motor_controller.drive(speed)
        speed_event.clear()

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

    speed = 0
    speed_event = threading.Event()

    threading.Thread(target=drive_thread, args=(motor_controller, speed_event), daemon=True).start()

    try:
        while True:
            # showMessage(stdscr, f"Current value: {gyro.get_angle_z():.2f}")
            key = stdscr.getch()

            if key == curses.KEY_UP:
                speed = min(100, speed + 5)
                showMessage(stdscr, f'Moving Forward (Speed: {str(speed)})\n')
            elif key == curses.KEY_DOWN:
                speed = max(-100, speed - 5)
                showMessage(stdscr, f'Moving Backward (Speed: {str(speed)})\n')
            elif key == curses.KEY_LEFT and not is_busy:
                showMessage(stdscr, 'Rotating Left\n')
                is_busy = True
                motor_controller.rotate_to_angle(gyro, target_angle=90, direction='left')
            elif key == curses.KEY_RIGHT and not is_busy:
                showMessage(stdscr, 'Rotating Right\n')
                is_busy = True
                motor_controller.rotate_to_angle(gyro, target_angle=90, direction='right')
            elif key == ord('d') and not is_busy:
                showMessage(stdscr, 'Reading distance\n')
                is_busy = True
                distance = sensor.get_distance()
                print(f"Distance sensor: {distance} cm")
            elif key == ord('m') and not is_busy:
                showMessage(stdscr, 'Map saved as: map.png\n')
                is_busy = True
                motor_controller.mapper.save_map_as_txt()
                motor_controller.mapper.save_map_as_png()
            elif key == ord('a') and not is_busy:
                stdscr.clear()
                stdscr.refresh()
                print(f"gyro.get_angle_z(): {gyro.get_angle_z()}")
                is_busy = True
            elif key == ord('q'):
                showMessage(stdscr, 'Quitting')
                break

            speed_event.set()
            speed_event.wait()

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
