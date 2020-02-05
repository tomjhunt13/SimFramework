#include <iostream>

#include "Eigen/Dense"

//#include "../src/StateSpace.h"
//#include "../src/RK4.h"

//class MassSpringDamper {
//
//private:
//    StateSpace* model;
//    float m = 1.f;
//    float k = 0.2;
//    float c = 0.01;
//
//public:
//    MassSpringDamper() {
//        // State space matrices
//        Eigen::MatrixXf A(2, 2);
//        A << 0, 1.f, -k, -c;
//
//        Eigen::MatrixXf B(2, 1);
//        B << 0, 1 / m;
//
//        Eigen::MatrixXf C(1, 2);
//        C << 1, 0;
//
//        Eigen::MatrixXf D(2, 1);
//        D << 0, 0;
//
//        Eigen::VectorXf xInit(2);
//        xInit << 0, 0;
//
//        Eigen::VectorXf u(1);
//        u << 0;
//
//        this->model = new StateSpace(A, B, C, D, xInit, u);
//    }
//
//    ~MassSpringDamper() { delete this->model; };
//
//    void Step() {
//        this->model->Step(0, 0.5);
//    }
//
//
//};

struct a {
    int sdfgh = 3;
};

int main() {

    a st;
    std::cout << st.sdfgh << std::endl;
//    MassSpringDamper msd;
//    msd.Step();

    return 0;
}