#include <iostream>

#include "Eigen/Dense"

#include "../src/StateSpace.h"
#include "../src/RK4.h"
#include "../src/Signal.h"


int main() {


    // Define signals
    Eigen::VectorXf in(1);
    in << 2;
    Eigen::VectorXf out(1);
    in << 0;

    Signal<Eigen::VectorXf> inputSignal(in);
    Signal<Eigen::VectorXf> outputSignal(out);

    // State space matrices
    float m = 1.f;
    float k = 0.2;
    float c = 0.01;

    Eigen::MatrixXf A(2, 2);
    A << 0, 1.f, -k, -c;

    Eigen::MatrixXf B(2, 1);
    B << 0, 1 / m;

    Eigen::MatrixXf C(1, 2);
    C << 1, 0;

    Eigen::MatrixXf D(2, 1);
    D << 0, 0;

    // Fill struct
    StateSpaceModel ss;
    ss.inputSignal = &inputSignal;
    ss.outputSignal = &outputSignal;
    ss.A = A;
    ss.B = B;
    ss.C = C;
    ss.D = D;

    // Instantiate system
    Eigen::VectorXf initialState(2);
    initialState << 0, 0.5;
    StateSpace block(ss, initialState,  0.f);

    // Test system
    std::cout << block.Gradient(0, initialState) << std::endl;

    return 0;
}