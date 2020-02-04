//
// Created by Thomas Hunt on 04/02/2020.
//

#include "StateSpace.h"

Eigen::VectorXf StateSpace::compute(Eigen::VectorXf u) {

    // x dot
    return *(this->m_A) * *(this->m_x) + *(this->m_B) * u;

    // y
    return *(this->m_C) * *(this->m_x) + *(this->m_D) * u;

};