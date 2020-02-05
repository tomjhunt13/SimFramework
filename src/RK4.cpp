//
// Created by Thomas Hunt on 04/02/2020.
//

#include "RK4.h"

Eigen::VectorXf RK4::Step(DynamicSystem& system, float dt, float t, Eigen::VectorXf& x) {

    Eigen::VectorXf k1 = system.Gradient(t, x);
    Eigen::VectorXf k2 = system.Gradient(t + dt / 2.f, x + k1 / 2.f);
    Eigen::VectorXf k3 = system.Gradient(t + dt / 2.f, x + k2 / 2.f);
    Eigen::VectorXf k4 = system.Gradient(t + dt, x + k3);

    return x + (1.f / 6.f) * (k1 + 2.f * k2 + 2.f * k3 + k4);
}
