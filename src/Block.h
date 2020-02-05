#ifndef SIMINTERFACE_BLOCK_H
#define SIMINTERFACE_BLOCK_H

#include <vector>
#include <string>

#include "Signal.h"

class Block {

private:
//    // Admin
//    std::string m_Name;

    // Signals
    std::vector<Signal*> m_InputSignals;
    Signal* m_OutputSignal;

public:

    Block(std::vector<Signal*>& inputSignals, Signal* outputSignal) : m_InputSignals(inputSignals), m_OutputSignal(outputSignal) {};

};


#endif //SIMINTERFACE_BLOCK_H
