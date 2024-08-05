#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>

// Constants
const double g = 40;  // Gravity
const double L1 = 100.0;  // Length of first pendulum 
const double L2 = 100.0;  // Length of second pendulum 
const double m1 = 1.0;  // Mass of first pendulum
const double m2 = 1.0;  // Mass of second pendulum
const double I1 = (1.0 / 20) * m1 * L1 * L1;  // Moment of inertia of first pendulum
const double I2 = (1.0 / 20) * m2 * L2 * L2;  // Moment of inertia of second pendulum
const double damping1 = 2;  // Damping coefficient for first pendulum
const double damping2 = 2;  // Damping coefficient for second pendulum

// Function prototypes
void rungeKutta(double* state, double t, double dt);
void derivatives(double* state, double* dstate, double t);
sf::Vector2f calculatePendulumPosition(double L, double theta, const sf::Vector2f& origin);
void handleUserInput(sf::RenderWindow& window, double* state, bool& isPaused, bool& isDragging1, bool& isDragging2, sf::Vector2f& dragOffset, const sf::Vector2f& origin);

// Main function
int main() {
    double t = 0.0;
    double dt = 0.2;  
    double state[4] = { M_PI / 2, 0, M_PI / 2, 0 }; // initial conditions: theta1, omega1, theta2, omega2

    // Create the main window
    sf::RenderWindow window(sf::VideoMode(800, 600), "Double Pendulum");
    bool isPaused = true;
    bool isDragging1 = false;
    bool isDragging2 = false;
    sf::Vector2f dragOffset(0, 0);
    sf::Vector2f origin(400, 300);

    // Vector to store the trail positions
    std::vector<sf::Vector2f> trail;

    // Clock to keep track of elapsed time
    sf::Clock clock;
    sf::Time elapsedTime = sf::Time::Zero;

    // Load font
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        std::cerr << "Error loading font\n";
        return -1;
    }

    // Text to display elapsed time
    sf::Text timerText;
    timerText.setFont(font);
    timerText.setCharacterSize(24);
    timerText.setFillColor(sf::Color::White);
    timerText.setPosition(650, 10);

    while (window.isOpen()) {
        // Process events
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        handleUserInput(window, state, isPaused, isDragging1, isDragging2, dragOffset, origin);

        if (!isPaused && !isDragging1 && !isDragging2) {
            // Perform Runge-Kutta integration
            rungeKutta(state, t, dt);
            t += dt;

            // Update elapsed time
            elapsedTime += clock.restart();
        } else {
            // If paused, reset the clock to not accumulate time
            clock.restart();
        }

        // Convert angles to coordinates
        double theta1 = state[0];
        double theta2 = state[2];
        sf::Vector2f p1 = calculatePendulumPosition(L1, theta1, origin);
        sf::Vector2f p2 = calculatePendulumPosition(L2, theta2, p1);

        // Add the current position of the blue mass to the trail
        trail.push_back(p2);

        // Clear screen
        window.clear();

        // Draw the trail
        for (const auto& point : trail) {
            sf::CircleShape trailPoint(2);
            trailPoint.setOrigin(1, 1);
            trailPoint.setPosition(point);
            trailPoint.setFillColor(sf::Color::Red);
            window.draw(trailPoint);
        }

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

        // Update the timer text
        timerText.setString("Time: " + std::to_string(elapsedTime.asSeconds()));
        window.draw(timerText);

        // Update the window
        window.display();

        // Add delay to slow down the rendering speed
        sf::sleep(sf::milliseconds(16)); 
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
    for (int i = 0; i < 4; ++i) temp[i] = state[i] + dt * k3[i];
    derivatives(temp, k4, t + dt);

    for (int i = 0; i < 4; ++i) {
        state[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
}

/// Derivatives function
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
                 (m1 + m2) * g * sin(theta1) - damping1 * omega1) / den1;

    dstate[2] = omega2;

    dstate[3] = (-m2 * L2 * omega2 * omega2 * sin(delta) * cos(delta) +
                 (m1 + m2) * g * sin(theta1) * cos(delta) -
                 (m1 + m2) * L1 * omega1 * omega1 * sin(delta) -
                 (m1 + m2) * g * sin(theta2) - damping2 * omega2) / den2;
}


// Calculate pendulum position based on length and angle
sf::Vector2f calculatePendulumPosition(double L, double theta, const sf::Vector2f& origin) {
    return sf::Vector2f(origin.x + L * sin(theta), origin.y + L * cos(theta));
}

// Handle user input for interactivity
void handleUserInput(sf::RenderWindow& window, double* state, bool& isPaused, bool& isDragging1, bool& isDragging2, sf::Vector2f& dragOffset, const sf::Vector2f& origin) {
    sf::Vector2i mousePos = sf::Mouse::getPosition(window);

    // Calculate pendulum positions
    sf::Vector2f p1 = calculatePendulumPosition(L1, state[0], origin);
    sf::Vector2f p2 = calculatePendulumPosition(L2, state[2], p1);

    // Check if user is dragging one of the balls
    if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
        if (!isDragging1 && !isDragging2) {
            if (std::hypot(mousePos.x - p1.x, mousePos.y - p1.y) < 10) {
                isDragging1 = true;
                dragOffset = sf::Vector2f(mousePos) - p1;
                isPaused = true;
            } else if (std::hypot(mousePos.x - p2.x, mousePos.y - p2.y) < 10) {
                isDragging2 = true;
                dragOffset = sf::Vector2f(mousePos) - p2;
                isPaused = true;
            }
        } else if (isDragging1) {
            sf::Vector2f newP1 = sf::Vector2f(mousePos) - dragOffset;
            double dx = newP1.x - origin.x;
            double dy = newP1.y - origin.y;
            state[0] = std::atan2(dx, dy);
        } else if (isDragging2) {
            sf::Vector2f newP2 = sf::Vector2f(mousePos) - dragOffset;
            double dx = newP2.x - p1.x;
            double dy = newP2.y - p1.y;
            state[2] = std::atan2(dx, dy);
        }
    } else {
        isDragging1 = false;
        isDragging2 = false;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
            isPaused = !isPaused;
        }
    }
}

// To Run
//g++ JayDoublePendulum.cpp -o prog -I/opt/homebrew/Cellar/sfml/2.6.1/include -L/opt/homebrew/Cellar/sfml/2.6.1/lib -lsfml-graphics -lsfml-window -lsfml-system -lsfml-network
