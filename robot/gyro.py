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

    def __init__(self, bus_number=1, address=0x68, calib_value=None):
        self.bus = smbus.SMBus(bus_number)
        self.address = address
        self.initialize()
        self.last_time = time.time()
        self.angle_z = 0.0
        self.alpha = 0.98
        self.gyro_z_offset = self.calibrate_gyro(calib_value)
        self.sensitivity = 131.0  # Default

    def initialize(self):
        # Włącz MPU-6050 i ustaw opcje konfiguracji
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)  # Przebudzenie MPU-6050
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)  # Ustawienia żyroskopu
        time.sleep(0.1)  # Krótkie opóźnienie na rozruch

    def calibrate_gyro(self, value):
        gyro_config = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if gyro_config == 0x08:  # ±500°/s
            self.sensitivity = 65.5
        elif gyro_config == 0x10:  # ±1000°/s
            self.sensitivity = 32.8
        elif gyro_config == 0x18:  # ±2000°/s
            self.sensitivity = 16.4

        if not value:
            print("Calibrating gyro...")
            num_samples = 1000
            offset_sum = 0.0
            for _ in range(num_samples):
                gz = self.read_raw_gyro_data()
                offset_sum += gz
                time.sleep(0.01)  # Czekaj krótko na każdy pomiar

            calib_value = offset_sum / num_samples
            print(f"\nCalibrated value: {calib_value}")
            time.sleep(1)
            return calib_value
        else:
            print(f"\nGyro manual value: {value}")
            time.sleep(1)
            return value

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
        gz_deg_s = gz / self.sensitivity

        # Obliczenie czasu od ostatniego pomiaru
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Kąt z żyroskopu
        gyro_angle_z = self.angle_z + gz_deg_s * dt

        # Filtr komplementarny
        self.angle_z = self.alpha * gyro_angle_z + (1.0 - self.alpha) * self.angle_z

        # Integracja, aby uzyskać kąt
        self.angle_z += gz_deg_s * dt

    def get_angle_z(self):
        self.update_angle()
        return self.angle_z

    def reset_angle(self):
        self.angle_z = 0.0
        self.last_time = time.time()
