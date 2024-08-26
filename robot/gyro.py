import smbus
import time

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

    def __init__(self, bus_number=1, address=0x68):
        self.bus = smbus.SMBus(bus_number)
        self.address = address
        self.initialize()
        self.last_time = time.time()
        self.angle_z = 0.0
        self.time_step = 0.01  # Czas kroków w sekundach (10ms)
        self.gyro_z_offset = self.calibrate_gyro()

    def initialize(self):
        # Włącz MPU-6050 i ustaw opcje konfiguracji
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)  # Przebudzenie MPU-6050
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)  # Ustawienia żyroskopu
        time.sleep(0.1)  # Krótkie opóźnienie na rozruch

    def calibrate_gyro(self):
        print("Calibrating gyro...")
        num_samples = 1000
        offset_sum = 0.0
        for _ in range(num_samples):
            gz = self.read_raw_gyro_data()
            offset_sum += gz
            time.sleep(0.01)  # Czekaj krótko na każdy pomiar
        return offset_sum / num_samples

    def read_raw_gyro_data(self):
        # Odczytaj surowe dane z żyroskopu
        gz = self._read_word_2c(self.GYRO_ZOUT_H)
        return gz

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
        # Przeliczenie wartości z odczytów z rejestrów na stopnie/s
        sensitivity = 131.0  # Sensitivity dla zakresu ±250°/s
        gz_deg_s = gz / sensitivity

        # Obliczenie czasu od ostatniego pomiaru
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Integracja, aby uzyskać kąt
        self.angle_z += gz_deg_s * dt

    def get_angle_z(self):
        self.update_angle()
        return self.angle_z

    def reset_angle(self):
        self.angle_z = 0.0
        self.last_time = time.time()
