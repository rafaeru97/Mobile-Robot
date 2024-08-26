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

    def read_rotation(self):
        gx, gy, gz = self.read_raw_gyro_data()
        # Przeliczenie wartości z odczytów z rejestrów na stopnie/s
        # MPU-6050 ma różne zakresy czułości, w tym przypadku używamy 250 deg/s
        sensitivity = 131.0  # Sensitivity dla zakresu ±250°/s
        gx_deg_s = gx / sensitivity
        gy_deg_s = gy / sensitivity
        gz_deg_s = gz / sensitivity
        return gx_deg_s, gy_deg_s, gz_deg_s

    def read_rotation_radians(self):
        gx_deg_s, gy_deg_s, gz_deg_s = self.read_rotation()
        # Przeliczenie z stopni/s na radiany/s
        gx_rad_s = math.radians(gx_deg_s)
        gy_rad_s = math.radians(gy_deg_s)
        gz_rad_s = math.radians(gz_deg_s)
        return gx_rad_s, gy_rad_s, gz_rad_s