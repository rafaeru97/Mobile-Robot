from robot import MotorController, Encoder, Gyro, DistanceSensor, Mapper
import time
import RPi.GPIO as GPIO
import sys
import curses
from http.server import SimpleHTTPRequestHandler, HTTPServer
import threading
import os
import logging

logging.basicConfig(filename='app_errors.log', level=logging.ERROR,
                    format='%(asctime)s - %(levelname)s - %(message)s')


def start_http_server():
    os.chdir("/home/pi/Desktop/Mobile-Robot")

    # Uruchomienie serwera HTTP
    port = 5000

    sys.stderr = open(os.devnull, 'w')

    handler = SimpleHTTPRequestHandler
    httpd = HTTPServer(("0.0.0.0", port), handler)

    print(f"Server is running at port: {port}\n")
    print(f"Check static ip by: hostname -I\n")
    httpd.serve_forever()


def run_server():
    server_thread = threading.Thread(target=start_http_server, name="ServerThread")
    server_thread.daemon = True
    server_thread.start()


run_server()  # Ręczne wywołanie funkcji

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Definiujemy zmienne współdzielone
speed = 0
rotate = 0
distance = 0
distance_reading = False
orientation = 0
encoder_distance = 0
motor_status = ""

# Synchronizacja danych za pomocą Lock
lock = threading.Lock()


def sensor_thread(sensor):
    global distance
    while True:
        with lock:
            if distance_reading:
                try:
                    distance = sensor.get_distance()
                except Exception as e:
                    print(f"Sensor thread error: {e}")
        time.sleep(0.05)

def toggle_distance_reading(sensor):
    global distance_reading
    distance_reading = not distance_reading
    sensor.set_status(distance_reading)

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


def print_gui_data(stdscr, speed, distance, orientation, rotate, motor_status, encoder, program_status=""):
    try:
        stdscr.clear()
        height, width = stdscr.getmaxyx()

        border = "*" * (width - 2)
        stdscr.addstr(0, 0, border)
        stdscr.addstr(height // 2 - 6, 0, "*" + " Mobile Robot - UI".center(width - 4) + "*")
        stdscr.addstr(height // 2 - 5, 0, "*" + f"Status: {program_status}".center(width - 4) + "*")
        stdscr.addstr(height - 1, 0, border)

        stdscr.addstr(height // 2 - 3, 2, f"Speed:        {speed:.2f} units")
        stdscr.addstr(height // 2 - 2, 2, f"Rotary Speed:  {rotate:.2f} units")
        stdscr.addstr(height // 2, 2, f"Motor Controller Status:  {motor_status}")

        dist_status = "OFF"
        if distance_reading:
            dist_status = "ON"

        stdscr.addstr(height // 2 + 2, 2, f"Distance Sensor:     {distance:.2f} cm ({dist_status})")
        stdscr.addstr(height // 2 + 3, 2, f"Gyroscope Orientation:  {orientation:.2f} degrees")
        stdscr.addstr(height // 2 + 4, 2, f"Encoder Distance:  {encoder:.2f} meters")

        stdscr.addstr(height - 1, 2, "Press Ctrl+C to exit.")
        stdscr.refresh()
    except Exception as e:
        stdscr.addstr(0, 0, f"Error: {str(e)}")
        stdscr.refresh()


def main(stdscr):
    try:
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
        sensor_t = threading.Thread(target=sensor_thread, args=(sensor,), name="DistanceSensorThread")
        motor_t = threading.Thread(target=motor_control_thread, args=(motor_controller,), name="MotorControllerThread")
        gyro_t = threading.Thread(target=gyro_thread, args=(gyro,), name="GyroThread")

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
                elif key == ord('d'):
                    toggle_distance_reading(sensor)
                elif key == ord('m'):
                    program_status = "refresh map"
                    print_gui_data(stdscr, speed, distance, orientation, rotate, motor_status, encoder_distance,
                                   program_status)
                    mapper.create_map()
                elif key == ord('o'):
                    program_status = "post processing map"
                    print_gui_data(stdscr, speed, distance, orientation, rotate, motor_status, encoder_distance,
                                   program_status)
                    mapper.process_detected_points()
                elif key == ord('s'):
                    program_status = "saving map"
                    print_gui_data(stdscr, speed, distance, orientation, rotate, motor_status, encoder_distance,
                                   program_status)
                    mapper.save_detected_points(filename="mapa.json")
                elif key == ord('l'):
                    program_status = "loading map"
                    print_gui_data(stdscr, speed, distance, orientation, rotate, motor_status, encoder_distance,
                                   program_status)
                    mapper.process_saved_points("mapa.json", output_filename="mapa_test.png")
                elif key == ord('q'):
                    break

                if distance <= 22 and speed > 0:
                    speed = 0

                mapper.update_position()

                program_status = "running"
                print_gui_data(stdscr, speed, distance, orientation, rotate, motor_status, encoder_distance,
                               program_status)

                time.sleep(0.1)

            except KeyboardInterrupt:
                break

    except Exception as e:
        logging.error("An error occurred", exc_info=True)
        raise  # Opcjonalnie ponownie zgłoś wyjątek, aby program zakończył działanie


if __name__ == '__main__':
    curses.wrapper(main)
