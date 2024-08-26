from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper, calculate_new_position, angle_to_radians
import time
import curses

def main(stdscr):
    # Inicjalizacja kontrolera silników i enkoderów
    motor_controller = MotorController()
    left_encoder = Encoder(pin_a=19, pin_b=26, wheel_diameter=0.08, ticks_per_revolution=960)
    right_encoder = Encoder(pin_a=16, pin_b=1, wheel_diameter=0.08, ticks_per_revolution=960)

    # Ustawienia terminala
    curses.curs_set(0)
    stdscr.nodelay(1)  # Tryb nieblokujący
    stdscr.clear()

    try:
        while True:
            key = stdscr.getch()

            if key == curses.KEY_UP:
                stdscr.addstr(0, 0, 'Moving Forward')
                motor_controller.forward_with_encoders(left_encoder, right_encoder, target_distance=0.1, base_speed=50)  # Przesuń na krótką odległość
            elif key == curses.KEY_DOWN:
                stdscr.addstr(0, 0, 'Moving Backward')
                motor_controller.backward_with_encoders(left_encoder, right_encoder, target_distance=0.1, base_speed=50)  # Przesuń na krótką odległość
            elif key == curses.KEY_LEFT:
                stdscr.addstr(0, 0, 'Turning Left')
                # Dodaj kod do skręcania w lewo (może być realizowane przez różne prędkości silników)
            elif key == curses.KEY_RIGHT:
                stdscr.addstr(0, 0, 'Turning Right')
                # Dodaj kod do skręcania w prawo (może być realizowane przez różne prędkości silników)
            elif key == ord('q'):
                break  # Wyjście z pętli i zakończenie programu

            stdscr.refresh()
            time.sleep(0.1)  # Krótkie opóźnienie dla zmniejszenia obciążenia CPU

    except KeyboardInterrupt:
        pass

    finally:
        motor_controller.cleanup()

if __name__ == '__main__':
    curses.wrapper(main)
