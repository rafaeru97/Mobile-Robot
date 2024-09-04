import smbus
import time
import math


class Gyro:
    # Adresy rejestrów MPU-6050
    PWR_MGMT_1 = 0x6B
    GYRO_CONFIG = 0x1B
    GYRO_XOUT_H = 0x43
    GYRO_XOUT_L = 0x44
    GYRO_YOUT_H = 0x45
    GYRO_YOUT_L = 0x46
    GYRO_ZOUT_H = 0x47
    GYRO_ZOUT_L = 0x48
    ACCEL_XOUT_H = 0x3B
    ACCEL_XOUT_L = 0x3C
    ACCEL_YOUT_H = 0x3D
    ACCEL_YOUT_L = 0x3E
    ACCEL_ZOUT_H = 0x3F
    ACCEL_ZOUT_L = 0x40

    def __init__(self, bus_number=1, address=0x68, calib_value=None):
        self.bus = smbus.SMBus(bus_number)
        self.address = address
        self.last_time = time.time()
        self.angle_z = 0.0
        self.alpha = 0.95  # Filtr komplementarny
        self.sensitivity = 131.0  # Domyślna wartość
        self.log_file = None  # Zainicjalizowane na None

        # Inicjalizacja
        self.initialize()
        self.open_log_file()

        # Kalibracja żyroskopu
        self.gyro_z_offset = self.calibrate_gyro(calib_value)

        # Kalibracja akcelerometru
        self.accel_error_x, self.accel_error_y = self.calibrate_accelerometer()

    def initialize(self):
        # Włącz MPU-6050 i ustaw opcje konfiguracji
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)  # Przebudzenie MPU-6050
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)  # Ustawienia żyroskopu
        time.sleep(0.1)  # Krótkie opóźnienie na rozruch

    def open_log_file(self):
        # Otwórz plik logu
        self.log_file = open("gyro_debug_log.txt", "w")
        self.log("Plik logu otwarty.")

    def log(self, message):
        if self.log_file:
            # Logowanie wiadomości do pliku
            self.log_file.write(message + "\n")
            self.log_file.flush()

    def calibrate_gyro(self, value):
        gyro_config = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if gyro_config == 0x08:  # ±500°/s
            self.sensitivity = 65.5
        elif gyro_config == 0x10:  # ±1000°/s
            self.sensitivity = 32.8
        elif gyro_config == 0x18:  # ±2000°/s
            self.sensitivity = 16.4

        if not value:
            self.log("Kalibruję żyroskop...")
            num_samples = 1000
            offset_sum = 0.0
            for _ in range(num_samples):
                gz = self.read_raw_gyro_data()
                offset_sum += gz
                time.sleep(0.01)  # Czekaj krótko na każdy pomiar

            calib_value = offset_sum / num_samples
            self.log(f"Wartość kalibracji: {calib_value}")
            time.sleep(1)
            return calib_value
        else:
            self.log(f"Ręczna wartość żyroskopu: {value}")
            time.sleep(1)
            return value

    def calibrate_accelerometer(self):
        # Kalibracja akcelerometru
        self.log("Kalibruję akcelerometr...")
        num_samples = 200
        accel_error_x_sum = 0.0
        accel_error_y_sum = 0.0

        for _ in range(num_samples):
            acc_x, acc_y, acc_z = self.read_accelerometer_data()
            accel_error_x_sum += math.atan2(acc_y, math.sqrt(acc_x ** 2 + acc_z ** 2)) * (180 / math.pi)
            accel_error_y_sum += math.atan2(-acc_x, math.sqrt(acc_y ** 2 + acc_z ** 2)) * (180 / math.pi)
            time.sleep(0.01)

        accel_error_x = accel_error_x_sum / num_samples
        accel_error_y = accel_error_y_sum / num_samples
        self.log(f"Błędy kalibracji akcelerometru: X={accel_error_x}, Y={accel_error_y}")
        return accel_error_x, accel_error_y

    def read_raw_gyro_data(self):
        # Odczytaj surowe dane z żyroskopu dla osi Z
        gz = self._read_word_2c(self.GYRO_ZOUT_H)
        return gz

    def read_accelerometer_data(self):
        # Odczytaj surowe dane z akcelerometru
        acc_x = self._read_word_2c(self.ACCEL_XOUT_H)
        acc_y = self._read_word_2c(self.ACCEL_YOUT_H)
        acc_z = self._read_word_2c(self.ACCEL_ZOUT_H)
        return acc_x, acc_y, acc_z

    def _read_word_2c(self, addr):
        # Odczytaj 16-bitowe słowo z dwóch rejestrów
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        val = (high << 8) + low
        # Konwersja na wartość ujemną
        if val >= 0x8000:
            val -= 0x10000
        return val

    def update_angle(self):
        gz = self.read_raw_gyro_data() - self.gyro_z_offset
        gz_deg_s = gz / self.sensitivity

        # Obliczenie czasu od ostatniego pomiaru
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Kąt z żyroskopu
        gyro_angle_z = self.angle_z + gz_deg_s * dt

        # Filtr komplementarny
        acc_angle_z = self.calculate_acc_angle()
        self.angle_z = self.alpha * gyro_angle_z + (1.0 - self.alpha) * acc_angle_z

        # Logowanie danych do pliku
        self.log(f"gyro_angle_z: {gyro_angle_z}, acc_angle_z: {acc_angle_z}, angle_z: {self.angle_z}")

    def calculate_acc_angle(self):
        acc_x, acc_y, acc_z = self.read_accelerometer_data()
        acc_x /= 16384.0  # Normalizacja dla ±2g
        acc_y /= 16384.0
        acc_z /= 16384.0
        acc_angle_z = math.atan2(acc_y, acc_x) * (180 / math.pi)  # Kąt wokół osi Z
        return acc_angle_z

    def get_angle_z(self):
        self.update_angle()
        return self.gyro_angle_z

    def reset_angle(self):
        self.angle_z = 0.0
        self.last_time = time.time()

    def close(self):
        # Zamknięcie pliku logu
        if self.log_file:
            self.log_file.write("Zamknięcie pliku logu.\n")
            self.log_file.close()