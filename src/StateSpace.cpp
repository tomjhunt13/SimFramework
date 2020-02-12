#include "StateSpace.h"


namespace SimFramework {
    Eigen::VectorXf StateSpace::Gradient(float t, Eigen::VectorXf x) {

        return this->m_StateSpace.A * this->m_x + this->m_StateSpace.B * this->m_u;

    };

    void StateSpace::Read() {
        this->m_u = this->m_StateSpace.inputSignal->Read();
    }

    void StateSpace::Write() {

        Eigen::VectorXf y = this->m_StateSpace.C * this->m_x + this->m_StateSpace.D * this->m_u;
        this->m_StateSpace.outputSignal->Write(y);

    }

    void StateSpace::Update(float finalTime) {

        // TODO: This implementation describes fixed step integrator and should be in RK4 class

        // Get time steps to make
        std::vector<float> timesteps = TimeSteps(this->m_t, finalTime, this->m_StateSpace.dt);

        // Iterate over timesteps and step system
        float t = this->m_t;
        for (int i = 0; i < timesteps.size(); i++) {

            // Step system
//            this->m_x = RK4::Step<Eigen::VectorXf>(*this, timesteps[i], t, this->m_x);
            t += timesteps[i];
        }

        // Update signals and state
        this->m_t = finalTime;

    };
} // namespace SimFramework