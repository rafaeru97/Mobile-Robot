from robot import MotorController, Encoder, Gyro, DistanceSensor
import pygame
import time

# Inicjalizacja pygame
pygame.init()
screen = pygame.display.set_mode((600, 400))  # Ustawienia rozmiaru ekranu
pygame.display.set_caption("Robot Control")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 36)

# Inicjalizacja komponentów robota
left_encoder = Encoder(pin_a=19, pin_b=26, wheel_diameter=0.08, ticks_per_revolution=960)
right_encoder = Encoder(pin_a=16, pin_b=1, wheel_diameter=0.08, ticks_per_revolution=960)
motor_controller = MotorController()
motor_controller.setEncoders(left_encoder, right_encoder)
gyro = Gyro()
sensor = DistanceSensor(trigger_pin=23, echo_pin=24)

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

        # Rysowanie na ekranie
        screen.fill((0, 0, 0))  # Czyszczenie ekranu
        draw_text(f'Speed: {speed}', (255, 255, 255), screen, 20, 20)
        draw_text(f'Rotation: {rotate}', (255, 255, 255), screen, 20, 60)
        draw_text(f'Gyro Angle: {gyro.get_angle_z():.2f}', (255, 255, 255), screen, 20, 100)
        pygame.display.flip()

        # Ustawienie liczby klatek na sekundę
        clock.tick(60)

    pygame.quit()

if __name__ == '__main__':
    main()
