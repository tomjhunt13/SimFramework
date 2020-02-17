#include "OutputBlock.h"

OutputBlock::OutputBlock(
        SimFramework::Signal<Eigen::Vector2f>* massStates,
        SimFramework::Signal<float>* force)
        : m_MassStates(massStates), m_Force(force) {};

void OutputBlock::Read()
{
    this->m_StatesCopy = this->m_MassStates->Read();
    this->m_ForceCopy = this->m_Force->Read();
}

void OutputBlock::Init(float t_0)
{
    this->t_n = t_0;
}

void OutputBlock::Update(float t_np1)
{
    std::cout << "Time: " << this->t_n << ", States: " << this->m_StatesCopy << ", Force: " << this->m_ForceCopy << std::endl;
    this->t_n =  t_np1;
};

