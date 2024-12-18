import pygame
import sys
from graphics import Rotating_Module, WHITE

# Screen dimensions and setup
WIDTH, HEIGHT = 1600, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("The Bloom - Virtual")

def main():
    # Initialize Pygame
    pygame.init()

    # Create an instance of Rotating_Module
    Rot1 = Rotating_Module(
        center_x=WIDTH // 2,
        center_y=HEIGHT // 2,
        hex_radius=200,
        circle_radius=146,
        start_angle=45,
        end_angle=270,
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

        # Update the display and set frame rate
        pygame.display.flip()
        clock.tick(60)  # Limit to 60 FPS

    # Quit Pygame when done
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
