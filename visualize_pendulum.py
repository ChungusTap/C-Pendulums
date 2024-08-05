import ctypes
import numpy as np
import pygame
import math

# Load the shared library
lib = ctypes.CDLL('./libpendulum.dylib')

# Define the argument and return types for the functions
lib.rungeKutta.argtypes = (ctypes.POINTER(ctypes.c_double), ctypes.c_double, ctypes.c_double)
lib.rungeKutta.restype = None

# Constants
L1 = 100.0  # Length of first pendulum
L2 = 100.0  # Length of second pendulum
origin = (400, 300)  # Origin point for the pendulum

# Initialize the state
state = np.array([np.pi / 2, 0, np.pi / 2, 0], dtype=np.double)
dt = 0.02  # Smaller dt for smoother animation

def calculate_pendulum_position(L, theta, origin):
    """Calculate pendulum position based on length and angle."""
    x = origin[0] + L * math.sin(theta)
    y = origin[1] + L * math.cos(theta)
    return (x, y)

def main():
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("Double Pendulum")

    clock = pygame.time.Clock()
    running = True

    # List to store the trail of the second pendulum
    trail = []

    # Initialize time
    t = 0.0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Perform Runge-Kutta integration
        lib.rungeKutta(state.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), ctypes.c_double(t), ctypes.c_double(dt))
        t += dt

        # Calculate positions
        theta1, theta2 = state[0], state[2]
        p1 = calculate_pendulum_position(L1, theta1, origin)
        p2 = calculate_pendulum_position(L2, theta2, p1)

        # Add the current position of the second pendulum to the trail
        trail.append(p2)
        if len(trail) > 1000:  # Limit trail length
            trail.pop(0)

        # Clear screen
        screen.fill((0, 0, 0))

        # Draw the trail
        for point in trail:
            pygame.draw.circle(screen, (255, 0, 0), (int(point[0]), int(point[1])), 2)

        # Draw the pendulum
        pygame.draw.line(screen, (255, 255, 255), origin, p1, 2)
        pygame.draw.line(screen, (255, 255, 255), p1, p2, 2)
        pygame.draw.circle(screen, (255, 0, 0), (int(p1[0]), int(p1[1])), 10)
        pygame.draw.circle(screen, (0, 0, 255), (int(p2[0]), int(p2[1])), 10)

        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        clock.tick(300)

    pygame.quit()

if __name__ == "__main__":
    main()


