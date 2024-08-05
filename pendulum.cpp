// pendulum.cpp

#include <cmath>

// Constants
const double g = 40;  // Gravity
const double L1 = 100.0;  // Length of first pendulum 
const double L2 = 100.0;  // Length of second pendulum 
const double m1 = 1.0;  // Mass of first pendulum
const double m2 = 1.0;  // Mass of second pendulum
const double damping1 = 2;  // Damping coefficient for first pendulum
const double damping2 = 2;  // Damping coefficient for second pendulum

extern "C" {
    void rungeKutta(double* state, double t, double dt);
    void derivatives(double* state, double* dstate, double t);
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
                 (m1 + m2) * g * sin(theta1) - damping1 * omega1) / den1;

    dstate[2] = omega2;

    dstate[3] = (-m2 * L2 * omega2 * omega2 * sin(delta) * cos(delta) +
                 (m1 + m2) * g * sin(theta1) * cos(delta) -
                 (m1 + m2) * L1 * omega1 * omega1 * sin(delta) -
                 (m1 + m2) * g * sin(theta2) - damping2 * omega2) / den2;
}
