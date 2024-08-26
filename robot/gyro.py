import smbus
import time
import math

class Gyro:
    # Adresy rejestrów MPU-6050
    PWR_MGMT_1 = 0x6B
    GYRO_XOUT_H = 0x43
    GYRO_XOUT_L = 0x44
    GYRO_YOUT_H = 0x45
    GYRO_YOUT_L = 0x46
    GYRO_ZOUT_H = 0x47
    GYRO_ZOUT_L = 0x48

    def __init__(self, bus_number=1, address=0x68):
        self.bus = smbus.SMBus(bus_number)
        self.address = address
        self.initialize()
        self.last_time = time.time()
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0

    def initialize(self):
        # Włącz MPU-6050 (domyślnie w trybie uśpienia po włączeniu zasilania)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)  # Przebudzenie MPU-6050
        time.sleep(0.1)  # Krótkie opóźnienie na rozruch

    def read_raw_gyro_data(self):
        # Odczytaj surowe dane z żyroskopu
        gx = self._read_word_2c(self.GYRO_XOUT_H)
        gy = self._read_word_2c(self.GYRO_YOUT_H)
        gz = self._read_word_2c(self.GYRO_ZOUT_H)
        return gx, gy, gz

    def _read_word_2c(self, addr):
        # Odczytaj 16-bitowe słowo z dwóch rejestrów
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        val = (high << 8) + low
        # Konwersja na wartość ujemną
        if val >= 0x8000:
            val -= 0x10000
        return val

    def update_angles(self):
        gx, gy, gz = self.read_raw_gyro_data()
        # Przeliczenie wartości z odczytów z rejestrów na stopnie/s
        sensitivity = 131.0  # Sensitivity dla zakresu ±250°/s
        gx_deg_s = gx / sensitivity
        gy_deg_s = gy / sensitivity
        gz_deg_s = gz / sensitivity

        # Obliczenie czasu od ostatniego pomiaru
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Integracja, aby uzyskać kąt
        self.angle_x += gx_deg_s * dt
        self.angle_y += gy_deg_s * dt
        self.angle_z += gz_deg_s * dt

    def get_angles(self):
        self.update_angles()
        return self.angle_x, self.angle_y, self.angle_z