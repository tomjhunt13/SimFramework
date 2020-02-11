#include "Mass1D.h"

Mass1D::Mass1D(SimInterface::Signal<float> &inputSpringForce, SimInterface::Signal<Eigen::Vector2f>& outputStates) :
                inputSpringForce(&inputSpringForce), outputStates(&outputStates)
{
    // Write initial states to output signals
    this->states  = this->outputStates->Read();
//    this->Read();

};

void Mass1D::Read()
{
    this->u = this->inputSpringForce->Read();
}

void Mass1D::Write()
{
    this->outputStates->Write(this->states);
};

void Mass1D::Update(float t_np1)
{

    // Get dt
    float dt = t_np1 - this->t_n;

    Eigen::Vector2f x_np1 = SimInterface::ForwardEuler::Step<Eigen::Vector2f>(*this, dt, this->t_n, this->states);

    this->states = x_np1;
    this->t_n = t_np1;

};

Eigen::Vector2f Mass1D::Gradient(float t, Eigen::Vector2f x)
{
    Eigen::Vector2f output = {x[1], (-1.f / this->mass) * this->u};
    return output;
};