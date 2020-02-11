#include "OutputBlock.h"



OutputBlock::OutputBlock(SimInterface::Signal<std::vector<float>>& massStates) : massStates(&massStates) {};


void OutputBlock::Read()
{
    this->value = this->massStates->Read();
};

void OutputBlock::Update(float t)
{
    std::cout << "Time: " << t << ", States: " << this->value[0] << ", " << this->value[1] << std::endl;
};

