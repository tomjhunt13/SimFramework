#include <iostream>

#include "../src/StateSpace.h"
#include "Eigen/Dense"

int main() {

    float m = 1.f;
    float k = 0.2;
    float c = 0.01;

    // State space matrices
    Eigen::MatrixXf A(2, 2);
    A << 0, 1.f, -k, -c;

    Eigen::MatrixXf B(2, 1);
    B << 0, 1 / m;

    Eigen::MatrixXf C(1, 2);
    C << 1, 0;

    Eigen::MatrixXf D(2, 1);
    D << 0, 0;

    Eigen::VectorXf xInit(2);
    xInit << 0, 0;

    Eigen::VectorXf u(1);
    u << 1.5;

    StateSpace ss(A, B, C, D, xInit);


    std::cout << ss.compute(u) << std::endl;



//    std::cout << A <<std::endl;
//    std::cout<<"Sum of 3 + 4 :"<<sum(3, 4)<<std::endl;
    return 0;
}