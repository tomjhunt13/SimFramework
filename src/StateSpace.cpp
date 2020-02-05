#include "StateSpace.h"


Eigen::VectorXf StateSpace::Gradient(float t, Eigen::VectorXf x) {

    // Read input
    const Eigen::VectorXf u = this->m_StateSpace.inputSignal->Read();

    return this->m_StateSpace.A * this->m_x + this->m_StateSpace.B * u;

};


void StateSpace::Update(float finalTime) {

    // Get time steps to make
    std::vector<float> timesteps = DynamicSystem::TimeSteps(this->m_t, finalTime, this->m_StateSpace.dt);

    // Iterate over timesteps and step system
    for (int i = 0; i < timesteps.size(); i++) {

        // Step system
        std::cout << timesteps[i] << std::endl;
    }

    // Update signals and state
    this->m_t = finalTime;


};