#include "Mass1D.h"

Mass1D::Mass1D(SimFramework::Signal* inputSpringForce, SimFramework::Signal* outputStates)
{
    this->RegisterInputSignal(inputSpringForce);
    this->RegisterOutputSignal(outputStates);

    // Set size of input and output copies
    this->m_InputCopy.resize(1);
};


void Mass1D::Update(float t_np1)
{

    // Get dt
    float dt = t_np1 - this->t_n;

    Eigen::VectorXf x_np1 = SimFramework::ForwardEuler::Step(*this, dt, this->t_n, this->m_OutputCopy);

    this->m_OutputCopy = x_np1;
    this->t_n = t_np1;

};

Eigen::VectorXf Mass1D::Gradient(float t, Eigen::VectorXf x)
{
    Eigen::Vector2f output = {x[1], (-1.f / this->mass) * this->m_InputCopy[0]};
    return output;
};