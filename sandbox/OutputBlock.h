#ifndef SIMINTERFACE_OUTPUTBLOCK_H
#define SIMINTERFACE_OUTPUTBLOCK_H

#include <iostream>
#include <vector>

#include "../src/Block.h"
#include "../src/Signal.h"

class OutputBlock : public SimInterface::Block {

private:

    SimInterface::Signal<std::vector<float>>* massStates;
    std::vector<float> value;

public:
    OutputBlock(SimInterface::Signal<std::vector<float>>& massStates);

    // Block functions
    void Read() override;
    void Update(float t) override;
    void Write() override {};

};


#endif //SIMINTERFACE_OUTPUTBLOCK_H
