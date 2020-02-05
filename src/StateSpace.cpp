#include "StateSpace.h"


//void StateSpace::Step(float t, float dt) {
//
//    std::cout << RK4::Step(*this, t, *(this->m_x)) << std::endl;
//
//}
//
Eigen::VectorXf StateSpace::Gradient(float t, Eigen::VectorXf x) {

    // Read input
    const Eigen::VectorXf u = this->m_StateSpace.inputSignal->Read();

    return this->m_StateSpace.A * this->m_x + this->m_StateSpace.B * u;

};