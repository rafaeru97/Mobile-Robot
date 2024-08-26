from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper, calculate_new_position, angle_to_radians
import time
import curses

def main(stdscr):
    # Inicjalizacja kontrolera silników, enkoderów i żyroskopu
    motor_controller = MotorController()
    left_encoder = Encoder(pin_a=19, pin_b=26, wheel_diameter=0.08, ticks_per_revolution=960)
    right_encoder = Encoder(pin_a=16, pin_b=1, wheel_diameter=0.08, ticks_per_revolution=960)
    gyro = Gyro()  # Inicjalizuj swój żyroskop

    # Włącz tryb nie-blokujący w curses
    stdscr.nodelay(1)
    stdscr.timeout(100)

    while True:
        key = stdscr.getch()

        if key == curses.KEY_UP:
            stdscr.addstr(0, 0, 'Moving Forward')
            motor_controller.forward_with_encoders(left_encoder, right_encoder, 0.1)
        elif key == curses.KEY_DOWN:
            stdscr.addstr(0, 0, 'Moving Backward')
            motor_controller.backward_with_encoders(left_encoder, right_encoder, 0.1)
        elif key == curses.KEY_LEFT:
            stdscr.addstr(0, 0, 'Rotating Left')
            motor_controller.rotate(gyro, target_angle=90, direction='left', speed=50)
        elif key == curses.KEY_RIGHT:
            stdscr.addstr(0, 0, 'Rotating Right')
            motor_controller.rotate(gyro, target_angle=90, direction='right', speed=50)
        elif key == ord('q'):
            stdscr.addstr(0, 0, 'Quitting')
            break

        # Czyszczenie ekranu i rysowanie nowych danych
        stdscr.refresh()

    motor_controller.stop()
    motor_controller.cleanup()

    except KeyboardInterrupt:
        pass

    finally:
        motor_controller.cleanup()

if __name__ == '__main__':
    curses.wrapper(main)
