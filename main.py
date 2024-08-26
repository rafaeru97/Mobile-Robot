from robot import MotorController, Encoder, Gyro
import time
import curses

def main(stdscr):
    # Inicjalizacja kontrolera silników, enkoderów i żyroskopu
    motor_controller = MotorController()
    left_encoder = Encoder(pin_a=19, pin_b=26, wheel_diameter=0.08, ticks_per_revolution=960)
    right_encoder = Encoder(pin_a=16, pin_b=1, wheel_diameter=0.08, ticks_per_revolution=960)
    gyro = Gyro()  # Inicjalizuj swój żyroskop

    # Flaga do zarządzania operacjami
    is_busy = False

    # Włącz tryb nie-blokujący w curses
    stdscr.nodelay(1)
    stdscr.timeout(100)

    try:
        while True:
            key = stdscr.getch()

            if key == curses.KEY_UP and not is_busy:
                stdscr.addstr(0, 0, 'Moving Forward')
                is_busy = True
                motor_controller.forward_with_encoders(left_encoder, right_encoder, 0.1)
                is_busy = False
            elif key == curses.KEY_DOWN and not is_busy:
                stdscr.addstr(0, 0, 'Moving Backward')
                is_busy = True
                motor_controller.backward_with_encoders(left_encoder, right_encoder, 0.1)
                is_busy = False
            elif key == curses.KEY_LEFT and not is_busy:
                stdscr.addstr(0, 0, 'Rotating Left')
                is_busy = True
                motor_controller.rotate_to_angle(gyro, target_angle=90, direction='left', speed=50)
                is_busy = False
            elif key == curses.KEY_RIGHT and not is_busy:
                stdscr.addstr(0, 0, 'Rotating Right')
                is_busy = True
                motor_controller.rotate_to_angle(gyro, target_angle=90, direction='right', speed=50)
                is_busy = False
            elif key == ord('q'):
                stdscr.addstr(0, 0, 'Quitting')
                break

            # Czyszczenie ekranu i rysowanie nowych danych
            stdscr.refresh()

    except KeyboardInterrupt:
        stdscr.addstr(1, 0, 'Interrupted by user.')
        stdscr.refresh()
        time.sleep(1)  # Daj chwilę na zobaczenie komunikatu
    finally:
        motor_controller.stop()
        motor_controller.cleanup()

if __name__ == '__main__':
    curses.wrapper(main)
