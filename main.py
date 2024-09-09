from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper
import time
import RPi.GPIO as GPIO
import sys
import curses
from http.server import SimpleHTTPRequestHandler, HTTPServer
import threading
import os

def start_http_server():
    os.chdir("/home/pi/Desktop/Mobile-Robot")

    # Uruchomienie serwera HTTP
    port = 5000

    sys.stderr = open(os.devnull, 'w')

    handler = SimpleHTTPRequestHandler
    httpd = HTTPServer(("0.0.0.0", port), handler)

    print(f"Serwer działa na http://localhost:{port}")
    httpd.serve_forever()

def run_server():
    server_thread = threading.Thread(target=start_http_server)
    server_thread.daemon = True
    server_thread.start()

run_server()  # Ręczne wywołanie funkcji

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Definiujemy zmienne współdzielone
speed = 0
rotate = 0
distance = 0
orientation = 0
encoder_distance = 0
motor_status = ""

# Synchronizacja danych za pomocą Lock
lock = threading.Lock()


def sensor_thread(sensor):
    global distance
    while True:
        try:
            with lock:
                distance = sensor.get_distance()
            time.sleep(0.05)
        except Exception as e:
            print(f"Sensor thread error: {e}")


def motor_control_thread(motor_controller):
    global speed, rotate, motor_status, encoder_distance
    while True:
        try:
            with lock:
                if rotate > 0:
                    motor_controller.turn_left(rotate)
                elif rotate < 0:
                    motor_controller.turn_right(abs(rotate))
                elif speed == 0 and rotate == 0:
                    motor_controller.stop()
                else:
                    motor_controller.drive(speed)

                motor_status = motor_controller.getStatus()
                encoder_distance = motor_controller.getEncoderDistance()
            time.sleep(0.05)
        except Exception as e:
            print(f"Motor control thread error: {e}")


def gyro_thread(gyro):
    global orientation
    while True:
        try:
            with lock:
                orientation = gyro.get_angle_z()
            time.sleep(0.05)
        except Exception as e:
            print(f"Gyro thread error: {e}")


def print_gui_data(stdscr, speed, distance, orientation, rotate, status, encoder):
    try:
        stdscr.clear()
        height, width = stdscr.getmaxyx()

        border = "*" * (width - 2)
        stdscr.addstr(0, 0, border)
        stdscr.addstr(height // 2 - 2, 0, "*" + " Mobile Robot - UI".center(width - 2) + "*")
        stdscr.addstr(height - 1, 0, border)

        stdscr.addstr(height // 2 - 1, 2, f"Speed:        {speed:.2f} units")
        stdscr.addstr(height // 2, 2, f"Distance:     {distance:.2f} cm")
        stdscr.addstr(height // 2 + 1, 2, f"Orientation:  {orientation:.2f} degrees")
        stdscr.addstr(height // 2 + 2, 2, f"Rotary:  {rotate:.2f} units")
        stdscr.addstr(height // 2 + 3, 2, f"Motor Controller Status:  {status}")
        stdscr.addstr(height // 2 + 4, 2, f"Encoder Distance:  {encoder:.2f} meters")

        stdscr.addstr(height - 1, 2, "Press Ctrl+C to exit.")
        stdscr.refresh()
    except Exception as e:
        stdscr.addstr(0, 0, f"Error: {str(e)}")
        stdscr.refresh()



def main(stdscr):
    global speed, rotate
    left_encoder = Encoder(pin_a=19, pin_b=26, wheel_diameter=0.08, ticks_per_revolution=960)
    right_encoder = Encoder(pin_a=16, pin_b=1, wheel_diameter=0.08, ticks_per_revolution=960)
    motor_controller = MotorController()
    motor_controller.setEncoders(left_encoder, right_encoder)
    gyro = Gyro()
    sensor = DistanceSensor(trigger_pin=23, echo_pin=24)
    mapper = Mapper(motor_controller, gyro, sensor)
    mapper.create_map()

    # Tworzymy wątki dla różnych zadań
    sensor_t = threading.Thread(target=sensor_thread, args=(sensor,))
    motor_t = threading.Thread(target=motor_control_thread, args=(motor_controller,))
    gyro_t = threading.Thread(target=gyro_thread, args=(gyro,))

    # Startujemy wątki
    sensor_t.start()
    motor_t.start()
    gyro_t.start()

    # Włączamy nodelay, aby curses działało nieblokująco
    stdscr.nodelay(1)
    stdscr.timeout(100)

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
            elif key == ord(' '):
                speed = 0
                rotate = 0
            elif key == ord('m'):
                mapper.create_map()
            elif key == ord('o'):
                mapper.process_detected_points(mapper.detected_points)
                mapper.generate_and_plot_map()
            elif key == ord('q'):
                break

            if distance <= 10 and speed > 0:
                speed = 0

            mapper.update_position()

            print_gui_data(stdscr, speed, distance, orientation, rotate, motor_status, encoder_distance)

            time.sleep(0.1)

        except KeyboardInterrupt:
            break


if __name__ == '__main__':
    curses.wrapper(main)
