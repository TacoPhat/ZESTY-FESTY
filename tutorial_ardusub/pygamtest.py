import pygame

# Init all pygame modules
pygame.init()

# Init the joystick module
pygame.joystick.init()

# Count the joysticks
joystick_count = pygame.joystick.get_count()
print(f"Number of joysticks detected: {joystick_count}")

# If any joystick is found, print its name
if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick name: {joystick.get_name()}")
else:
    print("No joystick detected.")
