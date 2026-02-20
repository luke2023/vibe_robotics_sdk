import pygame
pygame.init()

pygame.joystick.init()

joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joystick detected.")
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick detected: {joystick.get_name()}")
    
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                print([joystick.get_axis(i) for i in range(joystick.get_numaxes())])