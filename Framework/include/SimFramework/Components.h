#ifndef SIMINTERFACE_CONSTANTBLOCK_H
#define SIMINTERFACE_CONSTANTBLOCK_H

#include <vector>
#include "Eigen/Dense"
#include "Framework.h"
#include "Interpolation.h"

namespace SimFramework {

    template <typename SignalType>
    class ConstantBlock : public Source {
    public:
        ConstantBlock(std::string name="Constant") : Source(name) {};

        void Configure(Signal<SignalType>* outputSignal, SignalType value)
        {
            this->m_OutputSignal = outputSignal;
            this->m_Value = value;
        }

        std::vector<SignalBase*> InputSignals() override
        {
            return {};
        }

        std::vector<SignalBase*> OutputSignals() override
        {
            return {this->m_OutputSignal};
        }

        void Initialise(float t_0) override
        {
            this->m_OutputSignal->Write(this->m_Value);
        };

        void Update(float dt) override {};

    private:
        Signal<SignalType>* m_OutputSignal;
        SignalType m_Value;
    };


    template <typename ValueType>
    class Input : public Source
    {
    public:
        void Configure(Signal<ValueType>* outSignal, ValueType initialValue)
        {
            this->m_Signal = outSignal;
            this->m_SignalValue = initialValue;
        }

        std::vector<SignalBase*> InputSignals() override
        {
            return {};
        }

        std::vector<SignalBase*> OutputSignals() override
        {
            return {this->m_Signal};
        }

        void WriteValue(ValueType value)
        {
            this->m_SignalValue = value;
        };

        void Initialise(float t_0) override
        {
            this->Update(0.f);
        }

        void Update(float dt) override
        {
            this->m_Signal->Write(this->m_SignalValue);
        };

    private:
        Signal<ValueType>* m_Signal;
        ValueType m_SignalValue;
    };


    class TriggerFunction : public Source
    {
    public:
        TriggerFunction(std::string name = "Trigger Function");

        void Configure(Signal<float>* outputSignal);
        void Trigger();
        virtual float Evaluate(float t) = 0;

        std::vector<SignalBase*> InputSignals() override;
        std::vector<SignalBase*> OutputSignals() override;
        void Initialise(float t_0) override;
        void Update(float dt) override;

    protected:
        float t_end;
        float m_Default;

    private:
        float t_n;
        bool m_State;

        // Signals
        Signal<float>* m_Output;
    };


    template <typename InputType, typename OutputType, int InputLength, int StateLength, int OutputLength>
    class StateSpace : public DynamicSystem, public Integrable<Eigen::Vector<float, StateLength>>
    {
    public:
        StateSpace(std::string name = "State Space") : DynamicSystem(name) {};

        void Configure(Signal<InputType>* inputSignal, Signal<OutputType>* outputSignal)
        {
            this->m_InputSignal = inputSignal;
            this->m_OutputSignal = outputSignal;
        }

        void SetInitialConditions(Eigen::Vector<float, StateLength> initialValue)
        {
            this->m_InitialValue = initialValue;
        }

        void SetMatrices(Eigen::Matrix<float, StateLength, StateLength> A,
                         Eigen::Matrix<float, StateLength, InputLength> B,
                         Eigen::Matrix<float, OutputLength, StateLength> C,
                         Eigen::Matrix<float, OutputLength, InputLength> D)
        {
            this->m_A = A;
            this->m_B = B;
            this->m_C = C;
            this->m_D = D;
        }

        std::vector<SignalBase*> InputSignals() override
        {
            return {this->m_InputSignal};
        }

        std::vector<SignalBase*> OutputSignals() override
        {
            return {this->m_OutputSignal};
        }

        void Initialise(float t_0) override
        {
            this->t_n = t_0;
            this->m_States = this->m_InitialValue;

            OutputType seg = this->m_C * this->m_States;

            this->m_OutputSignal->Write(this->m_C * this->m_States);
        };

        void ReadInputs() override
        {
            this->m_InputCopy = this->m_InputSignal->Read();
        };

        void Update(float dt) override
        {
            Eigen::Vector<float, StateLength> x_np1 = SimFramework::RK4::Step<Eigen::Vector<float, StateLength>>(this, dt, this->t_n, this->m_States);

            this->m_States = x_np1;
            this->t_n = this->t_n + dt;

            // Output equation
            this->m_OutputSignal->Write(this->m_C * this->m_States + this->m_D * this->m_InputCopy);

        };

        Eigen::Vector<float, StateLength> Derivative(float t, Eigen::Vector<float, StateLength> x) override
        {
            return this->m_A * this->m_States + this->m_B * this->m_InputCopy;
        };

    private:

        // Signals
        Signal<InputType>* m_InputSignal;
        Signal<OutputType>* m_OutputSignal;

        // Work copies
        float t_n;
        Eigen::Vector<float, StateLength> m_InitialValue;
        Eigen::Vector<float, StateLength> m_States;
        InputType m_InputCopy;

        // State space matrices
        Eigen::Matrix<float, StateLength, StateLength> m_A;
        Eigen::Matrix<float, StateLength, InputLength> m_B;
        Eigen::Matrix<float, OutputLength, StateLength> m_C;
        Eigen::Matrix<float, OutputLength, InputLength> m_D;
    };


    template <typename SignalType>
    class SummingJunction : public Function {
    public:
        SummingJunction(std::string name = "Summing Junction") : Function(name) {};

        void Configure(std::vector<Signal<SignalType>*> inputSignals,
                       Signal<SignalType>* outputSignal,
                       std::vector<float> weights)
        {
            this->m_InputSignals = inputSignals;
            this->m_OutputSignal = outputSignal;
            this->m_Weights = weights;
            this->m_InputCopies.resize(inputSignals.size());
            this->m_WeightedValues.resize(inputSignals.size());
        }

        std::vector<SignalBase*> InputSignals() override
        {
            std::vector<SignalBase*> signals(this->m_InputSignals.size());
            for (int i=0; i < this->m_InputSignals.size(); i++)
            {
                signals[i] = m_InputSignals[i];
            }
            return signals;
        }

        std::vector<SignalBase*> OutputSignals() override
        {
            return {this->m_OutputSignal};
        }

        void Update() override
        {
            // Read input signals
            for (int i = 0; i < this->m_InputSignals.size(); i++)
            {
                this->m_InputCopies[i] = this->m_InputSignals[i]->Read();
            }

            // Multiply inputs by weights
            for (int i = 0; i < this->m_InputSignals.size(); i++)
            {
                this->m_WeightedValues[i] = this->m_Weights[i] * this->m_InputCopies[i];
            }

            // Sum result and write solution
            if (!this->m_InputSignals.empty())
            {
                SignalType output = this->m_WeightedValues[0];

                for (int i = 1; i < this->m_InputSignals.size(); i++)
                {
                    output += this->m_WeightedValues[i];
                };

                // Write output signal
                this->m_OutputSignal->Write(output);
            }
        };

    private:
        // Parameters
        std::vector<float> m_Weights;

        // Signals
        std::vector<Signal<SignalType>*> m_InputSignals;
        Signal<SignalType>* m_OutputSignal;

        // Work variables
        std::vector<SignalType> m_InputCopies;
        std::vector<SignalType> m_WeightedValues;
    };


    template <typename InputType, typename OutputType>
    class Vectorise : public Function
    {
    public:
        Vectorise(std::string name = "Vectorise") : Function(name) {};

        void Configure(std::vector<Signal<InputType>*> inputs, Signal<OutputType>* output)
        {
            this->m_InputSignals = inputs;
            this->m_OutputSignal = output;
            this->m_InputCopies.resize(inputs.size());
        }

        std::vector<SignalBase*> InputSignals() override
        {
            std::vector<SignalBase*> signals(this->m_InputSignals.size());
            for (int i=0; i < this->m_InputSignals.size(); i++)
            {
                signals[i] = m_InputSignals[i];
            }
            return signals;
        }

        std::vector<SignalBase*> OutputSignals() override
        {
            return {this->m_OutputSignal};
        }

        void Update() override
        {
            // Read input signals
            for (int i = 0; i < this->m_InputSignals.size(); i++)
            {
                this->m_InputCopies[i] = this->m_InputSignals[i]->Read();
            }

            // Fill output vector copy
            for (int i = 0; i < this->m_InputSignals.size(); i++)
            {
                this->m_OutputCopy[i] = m_InputCopies[i];
            }

            // Write vector copy to output
            this->m_OutputSignal->Write(this->m_OutputCopy);
        };

    private:
        // Signals
        std::vector<Signal<InputType>*> m_InputSignals;
        Signal<OutputType>* m_OutputSignal;

        // Work variables
        std::vector<InputType> m_InputCopies;
        OutputType m_OutputCopy;
    };


    template <typename InputType, typename OutputType>
    class Mask : public Function
    {
    public:
        Mask(std::string name = "Mask") : Function(name) {};

        void Configure(Signal<InputType>* inputSignal, std::vector<Signal<OutputType>*> maskedSignals, std::vector<int> maskIndices)
        {
            this->m_InputSignal = inputSignal;
            this->m_MaskedSignals = maskedSignals;
            this->m_MaskIndices = maskIndices;
        };

        std::vector<SignalBase*> InputSignals() override
        {
            return {this->m_InputSignal};
        }

        std::vector<SignalBase*> OutputSignals() override
        {
            std::vector<SignalBase*> signals(this->m_MaskedSignals.size());
            for (int i=0; i < this->m_MaskedSignals.size(); i++)
            {
                signals[i] = m_MaskedSignals[i];
            }
            return signals;
        }

        void Update() override
        {
            // Read input vector
            this->m_InputCopy = this->m_InputSignal->Read();

            // Interate over vector elements and write value to masked signal
            for (int i = 0; i < this->m_MaskIndices.size(); i++)
            {
                this->m_MaskedSignals[i]->Write(this->m_InputCopy[this->m_MaskIndices[i]]);
            }
        };

    private:
        // Parameters
        std::vector<int> m_MaskIndices;

        // Signals
        Signal<InputType>* m_InputSignal;
        std::vector<Signal<OutputType>*> m_MaskedSignals;

        // Work variables
        InputType m_InputCopy;
    };


    // TODO: Gain should have SetGain method
    template <typename InputType, typename ReturnType, typename GainType=float>
    class Gain : public Function
    {
    public:

        void Configure(Signal<InputType>* inputSignal, Signal<ReturnType>* outputSignal, GainType gain)
        {
            this->m_InputSignal = inputSignal;
            this->m_OutputSignal = outputSignal;
            this->m_Gain = gain;
        };

        std::vector<SignalBase*> InputSignals() override
        {
            return {this->m_InputSignal};
        }

        std::vector<SignalBase*> OutputSignals() override
        {
            return {this->m_OutputSignal};
        }

        void Update() override
        {
            this->m_OutputSignal->Write(this->m_Gain * this->m_InputSignal->Read());
        };

    private:
        // Parameters
        GainType m_Gain;

        // Signals
        Signal<InputType>* m_InputSignal;
        Signal<ReturnType>* m_OutputSignal;
    };


    class LookupTable2D : public Function
    {
    public:
        LookupTable2D(std::string name = "LookupTable2D");
        void Configure(Signal<float>* x, Signal<float>* y, Signal<float>* out);
        void SetTable(Table3D& table);

        std::vector<SignalBase*> InputSignals() override;
        std::vector<SignalBase*> OutputSignals() override;
        void Update() override;

    private:
        // Table
        Table3D m_Table;

        // Signals
        Signal<float>* m_X;
        Signal<float>* m_Y;
        Signal<float>* m_Out;
    };


    template <typename InputType>
    class LinearBlend : public Function
    {
    public:
        LinearBlend(std::string name="Linear Blend") : Function(name) {};

        void Configure(Signal<InputType>* input1, Signal<InputType>* input2, Signal<float>* alpha, Signal<InputType>* output)
        {
            this->m_Input1 = input1;
            this->m_Input2 = input2;
            this->m_Alpha = alpha;
            this->m_Output = output;
        }

        std::vector<SignalBase*> InputSignals() override
        {
            return {this->m_Input1, this->m_Input2, this->m_Alpha};
        }

        std::vector<SignalBase*> OutputSignals() override
        {
            return {this->m_Output};
        }

        void Update() override
        {
            float alpha = this->m_Alpha->Read();
            this->m_Output->Write((1 - alpha) * this->m_Input1->Read() + alpha * this->m_Input2->Read());
        };

    private:
        // Signals
        Signal<InputType>* m_Input1;
        Signal<InputType>* m_Input2;
        Signal<float>* m_Alpha;
        Signal<InputType>* m_Output;
    };


    template <typename ValueType>
    class Output : public Sink
    {
    public:
        void Configure(Signal<ValueType>* inSignal, ValueType initialValue)
        {
            this->m_Signal = inSignal;
            this->m_SignalCopy = initialValue;
        }

        ValueType ReadValue()
        {
            return this->m_SignalCopy;
        };

        std::vector<SignalBase*> InputSignals() override
        {
            return {this->m_Signal};
        }

        std::vector<SignalBase*> OutputSignals() override
        {
            return {};
        }

        void Update(float dt) override
        {
            this->m_SignalCopy = this->m_Signal->Read();
        };

    private:
        Signal<ValueType>* m_Signal;
        ValueType m_SignalCopy;
    };

}; // namespace SimFramework


#endif //SIMINTERFACE_CONSTANTBLOCK_H
