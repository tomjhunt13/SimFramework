#ifndef SIMINTERFACE_STATESPACE_H
#define SIMINTERFACE_STATESPACE_H

#include <iostream>

#include "Eigen/Dense"

#include "StateSpace.h"
#include "RK4.h"

class StateSpace : public SystemInterface {

private:

    // State space matrices
    Eigen::MatrixXf* m_A;
    Eigen::MatrixXf* m_B;
    Eigen::MatrixXf* m_C;
    Eigen::MatrixXf* m_D;

    // State
    Eigen::VectorXf* m_x;

    // Input
    Eigen::VectorXf* m_u;

public:

    // Constructor
    StateSpace(Eigen::MatrixXf& A, Eigen::MatrixXf& B, Eigen::MatrixXf& C, Eigen::MatrixXf& D,
                Eigen::VectorXf& initialState, Eigen::VectorXf& u) :
            m_A((&A)), m_B(&B), m_C(&C), m_D(&D), m_x(&initialState), m_u(&u) {};

    // Temporary evaluation of system
    Eigen::VectorXf compute(Eigen::VectorXf u);

    void Step(float t, float dt);

    Eigen::VectorXf Gradient(float t, Eigen::VectorXf x) override;

};

#endif //SIMINTERFACE_STATESPACE_H
