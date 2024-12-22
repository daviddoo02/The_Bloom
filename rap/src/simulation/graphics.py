import pygame
import math

# Screen dimensions and setup
WIDTH, HEIGHT = 1600, 800
# Colors
ORANGE = (255, 165, 0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

class Rotating_Module:
    def __init__(self, offset_x, offset_y, start_angle=0, end_angle=360, speed=2, direction=1):
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
        self.center_x = WIDTH // 2 + offset_x
        self.center_y = HEIGHT // 2 - offset_y
        self.hex_radius = 200/2
        self.circle_radius = 146/2
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

class Blooming_Module:
    def __init__(self, offset_x, offset_y, start_angle=0, end_angle=360, speed=2, direction=1):
        """
        Initialize the Blooming_Module object.
        
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
        self.center_x = WIDTH // 2 + offset_x
        self.center_y = HEIGHT // 2 - offset_y
        self.servo1_x = self.center_x - 40.35
        self.servo1_y = self.center_y + 0.0
        self.servo2_x = self.center_x + 40.35
        self.servo2_y = self.center_y + 0.0
        self.hex_radius = 200/2
        self.circle_radius = 146/4
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
        """Draw 2 wedges that sweeps between start_angle and current_angle."""
        if self.start_angle == self.current_angle:
            return  # Avoid drawing if there's no sweep

        points1 = [(self.servo1_x, self.servo1_y)]  # Start at servo1's center
        points2 = [(self.servo2_x, self.servo2_y)]  # Start at servo1's center

        # Add points along the arc between start_angle and current_angle
        for angle in range(self.start_angle, int(self.current_angle) + 1):
            angle_rad = math.radians(angle)
            x1 = self.servo1_x + self.circle_radius * math.cos(angle_rad)
            y1 = self.servo1_y + self.circle_radius * math.sin(angle_rad)
            x2 = self.servo2_x - self.circle_radius * math.cos(angle_rad)
            y2 = self.servo2_y - self.circle_radius * math.sin(angle_rad)
            points1.append((x1, y1))
            points2.append((x2, y2))

        # Ensure there are enough points to form a valid polygon
        if len(points1) > 2:
            pygame.draw.polygon(surface, BLUE, points1)
            pygame.draw.polygon(surface, BLUE, points2)

        

    def draw_sweeping_line(self, surface):
        """Draw a sweeping line around the circle."""
        # Calculate the endpoint of the line based on the current angle
        angle_rad = math.radians(self.current_angle)
        x1 = self.servo1_x + self.circle_radius * math.cos(angle_rad)
        y1 = self.servo1_y + self.circle_radius * math.sin(angle_rad)
        x2 = self.servo2_x - self.circle_radius * math.cos(angle_rad)
        y2 = self.servo2_y - self.circle_radius * math.sin(angle_rad)
            
        # Draw the line from the center to the calculated point
        pygame.draw.line(surface, BLACK, (self.servo1_x, self.servo1_y), (x1, y1), width=5)  # Width controls line thickness
        pygame.draw.line(surface, BLACK, (self.servo2_x, self.servo2_y), (x2, y2), width=5)  # Width controls line thickness

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
        
