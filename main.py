from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper
import time
import RPi.GPIO as GPIO
import curses

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def print_gui(stdscr, speed, distance, orientation, rotate, status, encoder):
    stdscr.clear()  # Czyści ekran
    height, width = stdscr.getmaxyx()  # Pobierz rozmiar terminala

    # Utwórz ramkę
    border = "*" * (width - 2)
    stdscr.addstr(0, 0, border)
    stdscr.addstr(height // 2 - 2, 0, "*" + " Mobile Robot - UI".center(width - 2) + "*")
    stdscr.addstr(height - 1, 0, border)

    # Wyświetl dane
    stdscr.addstr(height // 2 - 1, 2, f"Speed:        {speed:.2f} units")
    stdscr.addstr(height // 2, 2, f"Distance:     {distance:.2f} cm")
    stdscr.addstr(height // 2 + 1, 2, f"Orientation:  {orientation:.2f} degrees")
    stdscr.addstr(height // 2 + 2, 2, f"Rotary:  {rotate:.2f} units")
    stdscr.addstr(height // 2 + 3, 2, f"Motor Controller Status:  {status}")
    stdscr.addstr(height // 2 + 4, 2, f"Encoder Distance:  {encoder:.2f} meters")

    # Instrukcje
    stdscr.addstr(height - 1, 2, "Press Ctrl+C to exit.")

    stdscr.refresh()  # Odśwież ekran

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
    mapper = Mapper(motor_controller, gyro)

    speed = 0
    rotate = 0

    while True:
        try:
            key = stdscr.getch()

            if key == curses.KEY_UP:
                rotate = 0
                speed = min(100, max(30, speed + 5))
            elif key == curses.KEY_DOWN:
                rotate = 0
                speed = max(-100, min(-30, speed - 5))
            elif key == curses.KEY_LEFT:
                speed = 0
                rotate = min(100, max(35, rotate + 2))
            elif key == curses.KEY_RIGHT:
                speed = 0
                rotate = max(-100, min(-35, rotate - 2))
            elif key == ord('m'):
                mapper.create_map()
            elif key == ord(' '):
                speed = 0
                rotate = 0
                motor_controller.stop()
            elif key == ord('q'):
                break

            if sensor.get_distance() <= 10:
                if speed > 0:
                    speed = 0
                rotate = 0
                motor_controller.stop()

            if rotate > 0:
                motor_controller.turn_left(rotate)
            elif rotate < 0:
                motor_controller.turn_right(abs(rotate))
            else:
                motor_controller.drive(speed)

            mapper.update_position()
            print_gui(stdscr, speed, sensor.get_distance(), gyro.get_angle_z(), rotate, motor_controller.getStatus(), motor_controller.getEncoderDistance())
            time.sleep(0.05)  # Spowolnienie pętli

        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    curses.wrapper(main)
