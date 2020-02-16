#include "Components.h"

namespace SimFramework {

    //-------------------- Constant
    ConstantBlock::ConstantBlock(Signal* outputSignal, Eigen::VectorXf value) : m_Value(value)
    {
        this->RegisterOutputSignal(outputSignal);
        this->m_OutputCopy = value;
    };

    //------------------------ Summing Junction

    SummingJunction::SummingJunction(
            std::vector<Signal*> inputSignals, Signal* outputSignal, std::vector<float> weights)
            : m_Weights(weights), m_nInputs(inputSignals.size())
        {
            for (Signal* in : inputSignals)
            {
                this->RegisterInputSignal(in);
            }

            this->RegisterOutputSignal(outputSignal);

            this->m_InputCopy.resize(inputSignals.size());

        };

        void SummingJunction::Update(float t)
        {
            Eigen::VectorXf outputValue;

            for (int i = 0; i < this->m_nInputs; i++)
            {
                outputValue += this->m_Weights[i] * this->m_InputCopy[i];
            }

            this->m_OutputCopy = outputValue;
        };

}; // namespace SimFramework