#ifndef SIMINTERFACE_STATESPACE_H
#define SIMINTERFACE_STATESPACE_H

#include <iostream>
#include <vector>

#include "Eigen/Dense"

#include "StateSpace.h"
#include "RK4.h"


struct StateSpaceModel {

    // Input output
    std::vector<Signal*> inputSignal;
    Signal* outputSignal;

    // Integration parameters
    float dt = 0.1;

    // State space matrices
    Eigen::MatrixXf A;
    Eigen::MatrixXf B;
    Eigen::MatrixXf C;
    Eigen::MatrixXf D;
};


class StateSpace : public DynamicSystem {

private:

    // Matrices
    StateSpaceModel m_StateSpace;

    // State
    Eigen::VectorXf m_x;
    float m_t;

public:

    // Constructor
    StateSpace(StateSpaceModel stateSpace, Eigen::VectorXf initialState, float initialTime) :
            DynamicSystem(stateSpace.inputSignal, stateSpace.outputSignal), // Initialise parent
            m_StateSpace(stateSpace), m_x(initialState), m_t(initialTime) {};

//    // Temporary evaluation of system
//    Eigen::VectorXf compute(Eigen::VectorXf u);
//
//    void Step(float t, float dt);
//
//    Eigen::VectorXf Gradient(float t, Eigen::VectorXf x) override;

};

#endif //SIMINTERFACE_STATESPACE_H
