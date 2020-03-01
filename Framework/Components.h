#ifndef SIMINTERFACE_CONSTANTBLOCK_H
#define SIMINTERFACE_CONSTANTBLOCK_H

#include <vector>
#include "Eigen/Dense"
#include "Framework.h"
#include "Interpolation.h"

namespace SimFramework {

    //-------------------- Constant
    template <typename SignalType>
    class ConstantBlock : public Block {
    public:
        ConstantBlock(Signal<SignalType>* outputSignal, SignalType value) : m_OutputSignal(outputSignal), m_Value(value) {};

        // Block API
        void Read() override {};
        void Write() override
        {
            this->m_OutputSignal->Write(this->m_Value);
        };
        void Update(float t_np1) override {};

        void Init(float t_0) override
        {
            this->Write();
        };

    private:
        Signal<SignalType>* m_OutputSignal;
        SignalType m_Value;
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
            std::vector<SignalType> weightedValues;

            for (int i = 0; i < this->m_InputSignals.size(); i++)
            {
                weightedValues.push_back(this->m_Weights[i] * this->m_InputCopies[i]);
            }

            if (!this->m_InputSignals.empty())
            {
                SignalType outputValue =  weightedValues[0];

                for (int i = 1; i < this->m_InputSignals.size(); i++)
                {
                    outputValue += weightedValues[i];
                };

                this->m_OutputCopy = outputValue;
            }

        };

        void Init(float t_0) override {};

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

    template <typename inputType, typename outputType>
    class Mask : public Block
    {
    public:

        Mask(Signal<inputType>* inputSignal, std::vector<Signal<outputType>*> maskedSignals, std::vector<int> maskIndices)
                : m_InputSignal(inputSignal), m_MaskedSignals(maskedSignals), m_MaskIndices(maskIndices)
            {
                this->m_OutputCopies.resize(maskedSignals.size());
            };

        // Block API
        void Read() override
        {
            this->m_InputCopy = this->m_InputSignal->Read();
        };

        void Write() override
        {
            for (int i = 0; i < this->m_MaskedSignals.size(); i++)
            {
                this->m_MaskedSignals[i]->Write(this->m_OutputCopies[i]);
            }
        };

        void Update(float t_np1) override
        {
            for (int i = 0; i < this->m_MaskIndices.size(); i++)
            {
                this->m_OutputCopies[i] = this->m_InputCopy[this->m_MaskIndices[i]];
            }
        };

        void Init(float t_0) override {};

    private:
        // Signals
        Signal<inputType>* m_InputSignal;
        std::vector<Signal<outputType>*> m_MaskedSignals;

        // Copies
        inputType m_InputCopy;
        std::vector<outputType> m_OutputCopies;

        // Parameters
        std::vector<int> m_MaskIndices;

    };


    class LookupTable2D : public Block
    {
    public:
        LookupTable2D(Table3D& table, Signal<float>* x, Signal<float>* y, Signal<float>* out);

        // Block API
        void Read() override;
        void Write() override;
        void Update(float t_np1) override;
        void Init(float t_0) override;

    private:
        // Table
        Table3D m_Table;

        // Signals
        Signal<float>* m_X;
        Signal<float>* m_Y;
        Signal<float>* m_Out;

        // Copies
        float m_XCopy;
        float m_YCopy;
        float m_OutCopy;
    };


    template <typename valueType>
    class Input : public Block
    {
    public:
        Input(Signal<valueType>* outSignal, valueType initialValue) : m_Signal(outSignal), m_SignalCopy(initialValue) {};

        void WriteValue(valueType value)
        {
            this->m_SignalCopy = value;
        };

        // Block API
        void Read() override {};

        void Write() override
        {
            this->m_Signal->Write(this->m_SignalCopy);
        };

        void Update(float t_np1) override {};

        void Init(float t_0) override
        {
            this-Write();
        };

    private:
        Signal<valueType>* m_Signal;
        valueType m_SignalCopy;
    };


    template <typename valueType>
    class Output : public Block
    {
    public:
        Output(Signal<valueType>* inSignal, valueType initialValue) : m_Signal(inSignal), m_SignalCopy(initialValue) {};

        valueType ReadValue()
        {
            return this->m_SignalCopy;
        };

        // Block API
        void Read() override
        {
            this->m_SignalCopy = this->m_Signal->Read();
        };

        void Write() override {};

        void Update(float t_np1) override {};

        void Init(float t_0) override {};

    private:
        Signal<valueType>* m_Signal;
        valueType m_SignalCopy;
    };


    // TODO: Dimensions should be in template, state vector length j, input vector length i, output vector length  k
    //          Matrices are of size: A (j, j), B (j, i), C (k, j), D (k, i)
    template <typename InputType, typename OutputType>
    class StateSpace : public Block, public DynamicSystem<Eigen::VectorXf> {
    public:
        StateSpace(Signal<InputType>* inputSignal, Signal<OutputType>* outputSignal,
                   Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf D,
                   Eigen::VectorXf initialValue)
                   : m_InputSignal(inputSignal), m_OutputSignal(outputSignal),
                     m_InitialValue(initialValue), m_States(initialValue),
                     m_A(A), m_B(B), m_C(C), m_D(D) {};


        // Block API
        void Read() override
        {
            this->m_InputCopy = this->m_InputSignal->Read();
        };

        void Write() override
        {
            // Output equation
            this->m_OutputSignal->Write(this->m_C * this->m_States + this->m_D * this->m_InputCopy);
        };

        void Update(float t_np1) override
        {
            // Get dt
            float dt = t_np1 - this->t_n;

            Eigen::VectorXf x_np1 = SimFramework::RK4::Step<Eigen::VectorXf>(this, dt, this->t_n, this->m_States);

            this->m_States = x_np1;
            this->t_n = t_np1;

        };

        void Init(float t_0) override
        {
            this->t_n = t_0;
            this->m_States = this->m_InitialValue;
            this->m_OutputSignal->Write(this->m_States);
        };

        // Dynamic system functions
        Eigen::VectorXf Gradient(float t, Eigen::VectorXf x) override
        {
            return this->m_A * this->m_States + this->m_B * this->m_InputCopy;
        };

    private:

        // Signals
        Signal<InputType>* m_InputSignal;
        Signal<OutputType>* m_OutputSignal;

        // Work copies
        float t_n;
        Eigen::VectorXf m_InitialValue;
        Eigen::VectorXf m_States;
        InputType m_InputCopy;
//        Eigen::VectorXf m_OutputCopy;

        // State space matrices
        Eigen::MatrixXf m_A;
        Eigen::MatrixXf m_B;
        Eigen::MatrixXf m_C;
        Eigen::MatrixXf m_D;
    };
}; // namespace SimFramework


#endif //SIMINTERFACE_CONSTANTBLOCK_H
