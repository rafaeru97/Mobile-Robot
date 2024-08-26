import math

def calculate_new_position(current_position, distance, angle):
    x, y = current_position
    new_x = x + distance * math.cos(angle)
    new_y = y + distance * math.sin(angle)
    return (new_x, new_y)

def angle_to_radians(angle):
    return angle * (math.pi / 180)

