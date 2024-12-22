import pygame
import sys
from graphics import Rotating_Module, Blooming_Module, WHITE, WIDTH, HEIGHT

# Screen dimensions and setup
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("The Bloom - Virtual")

def main():
    # Initialize Pygame
    pygame.init()

    # Create instances of Rotating_Module
    Rot1 = Rotating_Module(
        offset_x= 0,
        offset_y= 0,
        start_angle=0,
        end_angle=200,
        speed=2,
        direction=1
    )

    Rot2 = Rotating_Module(
        offset_x= 150,
        offset_y= 86.6,
        start_angle=0,
        end_angle=200,
        speed=2,
        direction=1
    )

    Rot3 = Rotating_Module(
        offset_x= 0,
        offset_y= 173,
        start_angle=0,
        end_angle=200,
        speed=2,
        direction=1
    )

    Blm1 = Blooming_Module(
        offset_x= -149.93,
        offset_y= -86.6,
        start_angle=0,
        end_angle=200,
        speed=2,
        direction=1
    )

    Blm2 = Blooming_Module(
        offset_x= -299.93,
        offset_y= 0,
        start_angle=0,
        end_angle=200,
        speed=2,
        direction=1
    )

    Blm3 = Blooming_Module(
        offset_x= -449.93,
        offset_y= -86.6,
        start_angle=0,
        end_angle=200,
        speed=2,
        direction=1
    )

    # Main game loop
    clock = pygame.time.Clock()
    running = True
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Clear the screen
        screen.fill(WHITE)

        # Draw and update the hexagon with sweeping line animation
        Rot1.draw(screen)
        Rot1.update()
        Rot2.draw(screen)
        Rot2.update()
        Rot3.draw(screen)
        Rot3.update()
        Blm1.draw(screen)
        Blm1.update()
        Blm2.draw(screen)
        Blm2.update()
        Blm3.draw(screen)
        Blm3.update()

        # Update the display and set frame rate
        pygame.display.flip()
        clock.tick(60)  # Limit to 60 FPS

    # Quit Pygame when done
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
