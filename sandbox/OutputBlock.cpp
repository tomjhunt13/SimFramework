#include "OutputBlock.h"



OutputBlock::OutputBlock(SimFramework::Signal<Eigen::Vector2f>& massStates, SimFramework::Signal<float>& force)
                : massStates(&massStates), force(&force) {};


void OutputBlock::Read()
{
    this->states = this->massStates->Read();
    this->forceVal = this->force->Read();
};

void OutputBlock::Update(float t)
{
    std::cout << "Time: " << t << ", States: " << this->states << ", Force: " << this->forceVal << std::endl;
};

