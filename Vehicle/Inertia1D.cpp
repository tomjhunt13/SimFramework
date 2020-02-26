#include "Inertia1D.h"

namespace Vehicle {

    Inertia1D::Inertia1D(
            SimFramework::Signal<float> *inputSpringForce,
            SimFramework::Signal<Eigen::Vector2f> *outputStates,
            Eigen::Vector2f initialStates, float inertia)
            : m_InputForcing(inputSpringForce), m_OutputStates(outputStates), m_InitStates(initialStates), m_Inertia(inertia) {};


    void Inertia1D::Read() {
        this->m_InputCopy = this->m_InputForcing->Read();
    };

    void Inertia1D::Write() {
        this->m_OutputStates->Write(this->m_States);
    };

    void Inertia1D::Update(float t_np1) {

        // Get dt
        float dt = t_np1 - this->t_n;

        Eigen::Vector2f x_np1 = SimFramework::ForwardEuler::Step(*this, dt, this->t_n, this->m_States);

        this->m_States = x_np1;
        this->t_n = t_np1;

    };

    void Inertia1D::Init(float t_0) {
        this->t_n = t_0;
        this->m_States = this->m_InitStates;
        this->m_OutputStates->Write(this->m_States);
    }

    Eigen::Vector2f Inertia1D::Gradient(float t, Eigen::Vector2f x) {
        return {x[1], (1.f / this->m_Inertia) * this->m_InputCopy};
    };

}; // namespace Vehicle