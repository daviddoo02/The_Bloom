import pygame
import math

# Colors
ORANGE = (255, 165, 0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

class Rotating_Module:
    def __init__(self, center_x, center_y, hex_radius, circle_radius,
                 start_angle=0, end_angle=360, speed=2, direction=1):
        """
        Initialize the Rotating_Module object.
        
        Parameters:
            center_x (int): X-coordinate of the center.
            center_y (int): Y-coordinate of the center.
            hex_radius (int): Radius of the hexagon.
            circle_radius (int): Radius of the circle.
            start_angle (int): Starting angle for the sweeping line.
            end_angle (int): Ending angle for the sweeping line.
            speed (int): Speed of the sweeping animation in degrees per frame.
            direction (int): Initial direction of the sweep (1 for counterclockwise, -1 for clockwise).
        """
        self.center_x = center_x
        self.center_y = center_y
        self.hex_radius = hex_radius
        self.circle_radius = circle_radius
        self.start_angle = start_angle
        self.end_angle = end_angle
        self.speed = speed
        self.direction = direction
        self.current_angle = start_angle

    def draw_hexagon(self, surface):
        """Draw the hexagon with a black outline."""
        points = []
        for i in range(6):
            angle_rad = math.radians(i * 60)
            x = self.center_x + self.hex_radius * math.cos(angle_rad)
            y = self.center_y + self.hex_radius * math.sin(angle_rad)
            points.append((x, y))
        
        # Draw the filled hexagon
        pygame.draw.polygon(surface, ORANGE, points)
        
        # Draw the black outline
        pygame.draw.polygon(surface, BLACK, points, width=2)  # Width controls the thickness of the outline

    def draw_wedge(self, surface):
        """Draw a wedge that sweeps between start_angle and current_angle."""
        if self.start_angle == self.current_angle:
            return  # Avoid drawing if there's no sweep

        points = [(self.center_x, self.center_y)]  # Start at the circle's center

        # Add points along the arc between start_angle and current_angle
        for angle in range(self.start_angle, int(self.current_angle) + 1):
            angle_rad = math.radians(angle)
            x = self.center_x + self.circle_radius * math.cos(angle_rad)
            y = self.center_y + self.circle_radius * math.sin(angle_rad)
            points.append((x, y))

        # Ensure there are enough points to form a valid polygon
        if len(points) > 2:
            pygame.draw.polygon(surface, BLUE, points)

    def draw_sweeping_line(self, surface):
        """Draw a sweeping line around the circle."""
        # Calculate the endpoint of the line based on the current angle
        angle_rad = math.radians(self.current_angle)
        x = self.center_x + self.circle_radius * math.cos(angle_rad)
        y = self.center_y + self.circle_radius * math.sin(angle_rad)

        # Draw the line from the center to the calculated point
        pygame.draw.line(surface, BLACK, (self.center_x, self.center_y), (x, y), width=5)  # Width controls line thickness

    def update(self):
        """Update the current angle and reverse direction if needed."""
        # Update the current angle based on direction and speed
        self.current_angle += self.direction * self.speed

        # Reverse direction if it reaches the end or start angle
        if self.current_angle >= self.end_angle:
            self.direction = -1  # Switch to clockwise
            self.current_angle = self.end_angle
        elif self.current_angle <= self.start_angle:
            self.direction = 1   # Switch to counterclockwise
            self.current_angle = self.start_angle

    def draw(self, surface):
        """Draw all components: hexagon, wedge, and sweeping line."""
        self.draw_hexagon(surface)
        self.draw_wedge(surface)
        self.draw_sweeping_line(surface)
