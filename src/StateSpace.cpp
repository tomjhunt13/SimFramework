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
    float t = this->m_t;
    for (int i = 0; i < timesteps.size(); i++) {

        // Step system
        this->m_x = RK4::Step(*this, timesteps[i], t, this->m_x);
        t += timesteps[i];
    }

    // Update signals and state
    this->m_t = finalTime;
    this->m_StateSpace.outputSignal->Write(this->m_x);


};