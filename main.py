from robot import MotorController, Encoder, Gyro
import time
import curses

def showMessage(stdscr, message):
    stdscr.clear()
    stdscr.addstr(0, 0, message)
    stdscr.refresh()

def main(stdscr):
    # Inicjalizacja kontrolera silników, enkoderów i żyroskopu
    motor_controller = MotorController()
    left_encoder = Encoder(pin_a=19, pin_b=26, wheel_diameter=0.08, ticks_per_revolution=960)
    right_encoder = Encoder(pin_a=16, pin_b=1, wheel_diameter=0.08, ticks_per_revolution=960)
    gyro = Gyro(calib_value=-250)  # Inicjalizuj swój żyroskop

    # Flaga do zarządzania operacjami
    is_busy = False

    # Włącz tryb nie-blokujący w curses
    stdscr.nodelay(1)
    stdscr.timeout(100)

    try:
        while True:
            key = stdscr.getch()

            if key == curses.KEY_UP and not is_busy:
                showMessage(stdscr, 'Moving Forward\n')
                is_busy = True
                motor_controller.forward_with_encoders(left_encoder, right_encoder, 0.1)
            elif key == curses.KEY_DOWN and not is_busy:
                showMessage(stdscr, 'Moving Backward\n')
                is_busy = True
                motor_controller.backward_with_encoders(left_encoder, right_encoder, 0.1)
            elif key == curses.KEY_LEFT and not is_busy:
                showMessage(stdscr, 'Rotating Left\n')
                is_busy = True
                motor_controller.rotate_to_angle(gyro, target_angle=9, direction='left', speed=50)
            elif key == curses.KEY_RIGHT and not is_busy:
                showMessage(stdscr, 'Rotating Right\n')
                is_busy = True
                motor_controller.rotate_to_angle(gyro, target_angle=9, direction='right', speed=50)
            elif key == ord('q'):
                showMessage(stdscr, 'Quitting')
                break

            is_busy = False
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
