#include "SpringDamper1D.h"

SpringDamper1D::SpringDamper1D(
        SimFramework::Signal* inputConnection1,
        SimFramework::Signal* inputConnection2,
        SimFramework::Signal* outputForce)
{
    this->RegisterInputSignal(inputConnection1);
    this->RegisterInputSignal(inputConnection2);
    this->RegisterOutputSignal(outputForce);

    // Set size of input and output copies
    this->m_InputCopy.resize(2);
}

void SpringDamper1D::Update(float t)
{
    this->m_OutputCopy[0] = this->k * (this->m_InputCopy[1][0] - this->m_InputCopy[0][0]) + this->c * (this->m_InputCopy[1][1] - this->m_InputCopy[0][1]);
};