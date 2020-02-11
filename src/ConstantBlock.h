#ifndef SIMINTERFACE_CONSTANTBLOCK_H
#define SIMINTERFACE_CONSTANTBLOCK_H

#include "Block.h"
#include "Signal.h"

namespace SimInterface {

    template <typename dataType>
    class ConstantBlock :public Block {

    private:

        // Signal
        Signal<dataType>* outputSignal = nullptr;
        dataType value;

    public:
        ConstantBlock(Signal<dataType>& outputSignal, dataType value) : outputSignal(&outputSignal), value(value)
        {
            this->Write();
        };

        void Read() override {};
        void Write() override {};
        void Update(float finalTime) override {};

    };

}; // namespace SimInterface


#endif //SIMINTERFACE_CONSTANTBLOCK_H
