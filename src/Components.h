#ifndef SIMINTERFACE_CONSTANTBLOCK_H
#define SIMINTERFACE_CONSTANTBLOCK_H

#include "Eigen/Dense"
#include "Framework.h"

namespace SimFramework {

    //-------------------- Constant
    template <typename SignalType>
    class ConstantBlock : public Block {
    public:
        ConstantBlock(Signal<SignalType>* outputSignal, SignalType value)
        {
            outputSignal->Write(value);
        };

        // Block API
        void Read() override {};
        void Write() override {};
        void Update(float t_np1) override {};
    };


    //------------------------ Summing Junction
    template <typename SignalType>
    class SummingJunction : public Block {
    public:
        SummingJunction(
                std::vector<Signal<SignalType>*> inputSignals,
                Signal<SignalType>* outputSignal,
                std::vector<float> weights)
                : m_InputSignals(inputSignals), m_OutputSignal(outputSignal), m_Weights(weights)
        {
            this->m_InputCopies.reserve(inputSignals.size());
        };

        // Block API
        void Read() override
        {
            for (int i = 0; i < this->m_InputSignals.size(); i++)
            {
                this->m_InputCopies[i] = m_InputSignals[i]->Read();
            }
        };

        void Write() override
        {
            this->m_OutputSignal->Write(this->m_OutputCopy);
        };

        void Update(float t_np1) override
        {
            SignalType outputValue;

            for (int i = 0; i < this->m_InputSignals.size(); i++)
            {
                outputValue += this->m_Weights[i] * this->m_InputCopies[i];
            }

            this->m_OutputCopy = outputValue;

        };

    private:

        // Signals
        std::vector<Signal<SignalType>*> m_InputSignals;
        Signal<SignalType>* m_OutputSignal;

        // Internal copies
        std::vector<SignalType> m_InputCopies;
        SignalType m_OutputCopy;

        // Parameters
        std::vector<float> m_Weights;
    };

}; // namespace SimFramework


#endif //SIMINTERFACE_CONSTANTBLOCK_H
