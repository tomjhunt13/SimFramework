#include "Mass1D.h"

Mass1D::Mass1D(
        SimFramework::Signal<float>* inputSpringForce,
        SimFramework::Signal<Eigen::Vector2f>* outputStates,
        Eigen::Vector2f initialStates)
        : m_InputSpringForce(inputSpringForce), m_OutputStates(outputStates), m_States(initialStates) {};


void Mass1D::Read()
{
    this->m_InputCopy = this->m_InputSpringForce->Read();
};

void Mass1D::Write()
{
    this->m_OutputStates->Write(this->m_States);
};

void Mass1D::Update(float t_np1)
{

    // Get dt
    float dt = t_np1 - this->t_n;

    Eigen::Vector2f x_np1 = SimFramework::ForwardEuler::Step(*this, dt, this->t_n, this->m_States);

    this->m_States = x_np1;
    this->t_n = t_np1;

};

Eigen::Vector2f Mass1D::Gradient(float t, Eigen::Vector2f x)
{
    return {x[1], (-1.f / this->mass) * this->m_InputCopy};
};