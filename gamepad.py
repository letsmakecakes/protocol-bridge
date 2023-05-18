import time

import pygame


def game_loop():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No gamepad detected")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Handle gamepad input
        axis_x = joystick.get_axis(0)
        axis_y = joystick.get_axis(1)
        axis_z = joystick.get_axis(2)
        axis_p = joystick.get_axis(3)
        button_a = joystick.get_button(0)
        button_b = joystick.get_button(1)

        # Do something with the input
        print("Axis X:", axis_x)
        print("Axis Y:", axis_y)
        print("Axis z:", axis_z)
        print("Axis P:", axis_p)
        print("Button A:", button_a)
        print("Button B:", button_b)
        print("")
        time.sleep(1.5)

    pygame.quit()


game_loop()
