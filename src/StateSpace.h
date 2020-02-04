#ifndef SIMINTERFACE_STATESPACE_H
#define SIMINTERFACE_STATESPACE_H

#include "Eigen/Dense"

class StateSpace {

private:

    // State space matrices
    Eigen::MatrixXf* m_A;
    Eigen::MatrixXf* m_B;
    Eigen::MatrixXf* m_C;
    Eigen::MatrixXf* m_D;

    // State
    Eigen::VectorXf* m_x;

public:

    // Constructor
    StateSpace(Eigen::MatrixXf& A, Eigen::MatrixXf& B, Eigen::MatrixXf& C, Eigen::MatrixXf& D, Eigen::VectorXf& initialState) :
            m_A((&A)), m_B(&B), m_C(&C), m_D(&D), m_x(&initialState) {};

    // Temporary evaluation of system
    Eigen::VectorXf compute(Eigen::VectorXf u);

};


//using Eigen::MatrixXf;
////int main()
//{
//    MatrixXf m(2,2);
//    m(0,0) = 3;
//    m(1,0) = 2.5;
//    m(0,1) = -1;
//    m(1,1) = m(1,0) + m(0,1);
//    std::cout << m << std::endl;
//}

#endif //SIMINTERFACE_STATESPACE_H
