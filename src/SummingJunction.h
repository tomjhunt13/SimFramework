#ifndef SIMINTERFACE_SUMMINGJUNCTION_H
#define SIMINTERFACE_SUMMINGJUNCTION_H

#include <vector>



namespace SimFramework {

    template <typename SignalType>
    class SummingJunction : public Block {

    private:

        // Signals
        std::vector<Signal<SignalType>*> inputSignals;
        Signal<SignalType>* outputSignal;

        // Parameters
        std::vector<float> weights;

        // States
        std::vector<SignalType> values;
        SignalType output;


    public:
        SummingJunction(std::vector<Signal<SignalType>*> inputSignals,
                        Signal<SignalType>& outputSignal,
                        std::vector<float> weights) :
                        inputSignals(inputSignals), outputSignal(&outputSignal), weights(weights) {};

        // Block functions
        void Read() override
        {
            for (int i = 0; i < this->inputSignals.size(); i++)
            {
                this->values[i] = this->inputSignals[i]->Read();
            }
        };

        void Write() override
        {
            this->outputSignal->Write(this->output);
        };

        void Update(float t) override
        {
            SignalType outputValue;

            for (int i = 0; i < this->inputSignals.size(); i++)
            {
                outputValue += this->weights[i] * this->values[i];
            }

            this->output = outputValue;
        };

    };


} //  namespace SimFramework


#endif //SIMINTERFACE_SUMMINGJUNCTION_H
