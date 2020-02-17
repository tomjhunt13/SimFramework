#include "OutputBlock.h"



OutputBlock::OutputBlock(SimFramework::Signal* massStates, SimFramework::Signal* force)
{
    this->RegisterInputSignal(massStates);
    this->RegisterInputSignal(force);

    this->m_InputCopy.resize(2);
}



void OutputBlock::Update(float t_np1)
{
    std::cout << "Time: " << t_np1 << ", States: " << this->m_InputCopy[0] << ", Force: " << this->m_InputCopy[1] << std::endl;
};

