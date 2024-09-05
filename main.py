from robot import MotorController, Encoder, Gyro, DistanceSensor
import pygame
import time

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
    font = pygame.font.SysFont(None, 36)
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
                    try:
                        distance = sensor.get_distance()
                        print(f"Distance sensor: {distance} cm")
                    except Exception as e:
                        print(f"Error reading distance sensor: {e}")
                elif event.key == pygame.K_m:
                    try:
                        print('Map saved as: map.png')
                        motor_controller.mapper.save_map_as_txt()
                        motor_controller.mapper.save_map_as_png()
                    except Exception as e:
                        print(f"Error saving map: {e}")
                elif event.key == pygame.K_a:
                    try:
                        print(f"gyro.get_angle_z(): {gyro.get_angle_z()}")
                    except Exception as e:
                        print(f"Error reading gyro angle: {e}")
                elif event.key == pygame.K_q:
                    running = False

        # Aktualizacja robota
        try:
            motor_controller.drive(speed)
            if rotate > 0:
                motor_controller.rotate_to_angle(gyro, target_angle=rotate, direction='left')
            elif rotate < 0:
                rotate = abs(rotate)
                motor_controller.rotate_to_angle(gyro, target_angle=rotate, direction='right')
            rotate = 0
        except Exception as e:
            print(f"Error updating robot: {e}")

        # Rysowanie na ekranie
        screen.fill((0, 0, 0))  # Czyszczenie ekranu
        draw_text(f'Speed: {speed}', (255, 255, 255), screen, 20, 20)
        draw_text(f'Rotation: {rotate}', (255, 255, 255), screen, 20, 60)
        try:
            draw_text(f'Gyro Angle: {gyro.get_angle_z():.2f}', (255, 255, 255), screen, 20, 100)
        except Exception as e:
            print(f"Error reading gyro angle for display: {e}")
        pygame.display.flip()

        # Ustawienie liczby klatek na sekundę
        clock.tick(60)

    pygame.quit()

if __name__ == '__main__':
    main()
