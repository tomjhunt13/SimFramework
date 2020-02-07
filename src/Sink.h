//
// Created by Thomas Hunt on 05/02/2020.
//

#ifndef SIMINTERFACE_SINK_H
#define SIMINTERFACE_SINK_H


#include <iostream>

#include "Eigen/Dense"

#include "Block.h"
#include "Signal.h"

namespace SimInterface {

    class Sink : public Block {

    private:
        Signal<Eigen::VectorXf> *m_InputSignal;

    public:
        Sink(Signal<Eigen::VectorXf> *inputSignal) : m_InputSignal(inputSignal) {};

        void Read() override {};

        void Write() override {};

        void Update(float time) override;
    };

} // namespace SimInterface

#endif //SIMINTERFACE_SINK_H
