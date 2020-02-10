#include "Mass1D.h"


Mass1D::Mass1D(SimInterface::Signal<float> &inputSpringForce, SimInterface::Signal<std::vector<float>>& outputStates) :
                inputSpringForce(&inputSpringForce), outputStates(&outputStates)
{
    // Write initial states to output signals
    this->Write();

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

    std::vector<float> x_np1 = SimInterface::ForwardEuler::Step<std::vector<float>>(*this, dt, this->t_n, this->states);

};

std::vector<float> Mass1D::Gradient(float t, std::vector<float> x)
{
    std::vector<float> output = {x[1], (1 / this->mass) * this->u};
    return output;
};