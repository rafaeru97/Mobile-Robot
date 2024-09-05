from robot import MotorController, Encoder, Gyro, DistanceSensor
import time
import pygame
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

# Inicjalizacja pygame
try:
    pygame.init()
except Exception as e:
    print(f"Error initializing pygame: {e}")
    exit(1)

try:
    screen = pygame.display.set_mode((600, 400))  # Ustawienia rozmiaru ekranu
    pygame.display.set_caption("Robot Control")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 12)
except Exception as e:
    print(f"Error setting up pygame components: {e}")
    pygame.quit()
    exit(1)

# Inicjalizacja komponentów robota
try:
    left_encoder = Encoder(pin_a=19, pin_b=26, wheel_diameter=0.08, ticks_per_revolution=960)
    right_encoder = Encoder(pin_a=16, pin_b=1, wheel_diameter=0.08, ticks_per_revolution=960)
    motor_controller = MotorController()
    motor_controller.setEncoders(left_encoder, right_encoder)
    gyro = Gyro()
    sensor = DistanceSensor(trigger_pin=23, echo_pin=24)
except Exception as e:
    print(f"Error initializing robot components: {e}")
    pygame.quit()
    exit(1)

def draw_text(message, color, surface, x, y):
    text = font.render(message, True, color)
    surface.blit(text, (x, y))

def main():
    speed = 0
    rotate = 0
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    speed = min(100, speed + 5)
                elif event.key == pygame.K_DOWN:
                    speed = max(-100, speed - 5)
                elif event.key == pygame.K_LEFT:
                    speed = 0
                    rotate = min(30, rotate + 5)
                elif event.key == pygame.K_RIGHT:
                    speed = 0
                    rotate = max(-30, rotate - 5)
                elif event.key == pygame.K_d:
                    distance = sensor.get_distance()
                    print(f"Distance sensor: {distance} cm")
                elif event.key == pygame.K_m:
                    print('Map saved as: map.png')
                    motor_controller.mapper.save_map_as_txt()
                    motor_controller.mapper.save_map_as_png()
                elif event.key == pygame.K_a:
                    print(f"gyro.get_angle_z(): {gyro.get_angle_z()}")
                elif event.key == pygame.K_q:
                    running = False

        # Aktualizacja robota
        motor_controller.drive(speed)
        if rotate > 0:
            motor_controller.rotate_to_angle(gyro, target_angle=rotate, direction='left')
        elif rotate < 0:
            rotate = abs(rotate)
            motor_controller.rotate_to_angle(gyro, target_angle=rotate, direction='right')
        rotate = 0

        # Wydruk danych debugujących
        print(f'Speed: {speed}')
        print(f'Rotation: {rotate}')
        print(f'Gyro Angle: {gyro.get_angle_z():.2f}')
        time.sleep(0.1)  # Spowolnienie pętli

    pygame.quit()


if __name__ == '__main__':
    main()
