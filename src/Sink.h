//
// Created by Thomas Hunt on 05/02/2020.
//

#ifndef SIMINTERFACE_SINK_H
#define SIMINTERFACE_SINK_H


#include <iostream>

#include "Eigen/Dense"

#include "Block.h"
#include "Signal.h"

class Sink : public Block {

private:
    Signal<Eigen::VectorXf> m_InputSignal;

public:
    Sink(TimeManager& manager, Signal<Eigen::VectorXf> inputSignal) : Block(manager), m_InputSignal(inputSignal) {};

    void Update(float time) override;
};


#endif //SIMINTERFACE_SINK_H
