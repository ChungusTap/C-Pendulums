#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

// Constants
const double g = 9.81;  // Gravity
const double L1 = 200.0;  // Length of first pendulum (in pixels)
const double L2 = 200.0;  // Length of second pendulum (in pixels)
const double m1 = 1.0;  // Mass of first pendulum
const double m2 = 1.0;  // Mass of second pendulum

// Function prototypes
void rungeKutta(double* state, double t, double dt);
void derivatives(double* state, double* dstate, double t);

// Main function
int main() {
    double t = 0.0;
    double dt = 0.01;
    double state[4] = { M_PI / 2, 0, M_PI / 2, 0 }; // initial conditions: theta1, omega1, theta2, omega2

    // Create the main window
    sf::RenderWindow window(sf::VideoMode(800, 600), "Double Pendulum");

    while (window.isOpen()) {
        // Process events
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Clear screen
        window.clear();

        // Perform Runge-Kutta integration
        rungeKutta(state, t, dt);
        t += dt;

        // Convert angles to coordinates
        double theta1 = state[0];
        double theta2 = state[2];
        sf::Vector2f origin(400, 300);
        sf::Vector2f p1(origin.x + L1 * sin(theta1), origin.y + L1 * cos(theta1));
        sf::Vector2f p2(p1.x + L2 * sin(theta2), p1.y + L2 * cos(theta2));

        // Draw the pendulum
        sf::Vertex line1[] = { sf::Vertex(origin), sf::Vertex(p1) };
        sf::Vertex line2[] = { sf::Vertex(p1), sf::Vertex(p2) };
        sf::CircleShape ball1(10), ball2(10);
        ball1.setOrigin(10, 10);
        ball2.setOrigin(10, 10);
        ball1.setPosition(p1);
        ball2.setPosition(p2);
        ball1.setFillColor(sf::Color::Red);
        ball2.setFillColor(sf::Color::Blue);

        window.draw(line1, 2, sf::Lines);
        window.draw(line2, 2, sf::Lines);
        window.draw(ball1);
        window.draw(ball2);

        // Update the window
        window.display();
    }

    return 0;
}

// Runge-Kutta 4th order method
void rungeKutta(double* state, double t, double dt) {
    double k1[4], k2[4], k3[4], k4[4], temp[4];

    derivatives(state, k1, t);
    for (int i = 0; i < 4; ++i) temp[i] = state[i] + 0.5 * dt * k1[i];
    derivatives(temp, k2, t + 0.5 * dt);
    for (int i = 0; i < 4; ++i) temp[i] = state[i] + 0.5 * dt * k2[i];
    derivatives(temp, k3, t + 0.5 * dt);
    for (int i = 0; ++i < 4; i) temp[i] = state[i] + dt * k3[i];
    derivatives(temp, k4, t + dt);

    for (int i = 0; i < 4; ++i) {
        state[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
}

// Derivatives function
void derivatives(double* state, double* dstate, double t) {
    double theta1 = state[0];
    double omega1 = state[1];
    double theta2 = state[2];
    double omega2 = state[3];

    double delta = theta2 - theta1;
    double den1 = (m1 + m2) * L1 - m2 * L1 * cos(delta) * cos(delta);
    double den2 = (L2 / L1) * den1;

    dstate[0] = omega1;
    dstate[1] = (m2 * L1 * omega1 * omega1 * sin(delta) * cos(delta) +
                 m2 * g * sin(theta2) * cos(delta) +
                 m2 * L2 * omega2 * omega2 * sin(delta) -
                 (m1 + m2) * g * sin(theta1)) / den1;

    dstate[2] = omega2;
    dstate[3] = (-m2 * L2 * omega2 * omega2 * sin(delta) * cos(delta) +
                 (m1 + m2) * g * sin(theta1) * cos(delta) -
                 (m1 + m2) * L1 * omega1 * omega1 * sin(delta) -
                 (m1 + m2) * g * sin(theta2)) / den2;
}
