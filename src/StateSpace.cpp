//
// Created by Thomas Hunt on 04/02/2020.
//

#include "StateSpace.h"

//Eigen::VectorXf StateSpace::compute(Eigen::VectorXf u) {
//
//    // x dot
//    *(this->m_x) = (*(this->m_A) * *(this->m_x) + *(this->m_B) * u);
//
//    // y
//    return *(this->m_C) * *(this->m_x) + *(this->m_D) * u;
//
//};
//
//void StateSpace::Step(float t, float dt) {
//
//    std::cout << RK4::Step(*this, t, *(this->m_x)) << std::endl;
//
//}
//
//Eigen::VectorXf StateSpace::Gradient(float t, Eigen::VectorXf x) {
//
//    return *(this->m_x) = (*(this->m_A) * *(this->m_x) + *(this->m_B) * *(this->m_u));
//
//};