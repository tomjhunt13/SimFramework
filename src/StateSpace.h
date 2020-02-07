#ifndef SIMINTERFACE_STATESPACE_H
#define SIMINTERFACE_STATESPACE_H

#include <iostream>
#include <vector>

#include "Eigen/Dense"

#include "Utilities.h"

#include "Block.h"
#include "Signal.h"
#include "StateSpace.h"
#include "RK4.h"


namespace SimInterface {

    struct StateSpaceModel {

        // Input output
        Signal<Eigen::VectorXf> *inputSignal;
        Signal<Eigen::VectorXf> *outputSignal;

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
        Eigen::VectorXf m_u;
        float m_t;

    public:

        // Constructor
        StateSpace(BlockManager &manager, StateSpaceModel stateSpace, Eigen::VectorXf initialState, float initialTime) :
                DynamicSystem(manager), m_StateSpace(stateSpace), m_x(initialState), m_t(initialTime) {};


        Eigen::VectorXf Gradient(float t, Eigen::VectorXf x) override;

        void Read() override;

        void Update(float finalTime) override;

        void Write() override;


    };

} // namespace SimInterface

#endif //SIMINTERFACE_STATESPACE_H
