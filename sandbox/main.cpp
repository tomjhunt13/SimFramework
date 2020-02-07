#include <iostream>

#include "Eigen/Dense"

#include "SystemManager.h"
#include "Block.h"
#include "Signal.h"
#include "StateSpace.h"
#include "Sink.h"





int main() {


    // Define signals
    Eigen::VectorXf in(1);
    in << 0;
    Eigen::VectorXf out(1);
    out << 0;

    Signal<Eigen::VectorXf> inputSignal(in);
    Signal<Eigen::VectorXf> outputSignal(out);

    // State space matrices
    float m = 10.f;
    float k = 10;
    float c = 0.01;

    Eigen::MatrixXf A(2, 2);
    A << 0, 1.f, -k / m, -(c / m);

    Eigen::MatrixXf B(2, 1);
    B << 0, 1 / m;

    Eigen::MatrixXf C(1, 2);
    C << 1, 0;

    Eigen::MatrixXf D(1, 1);
    D << 0;

    // Fill struct
    SimInterface::StateSpaceModel ss;
    ss.inputSignal = &inputSignal;
    ss.outputSignal = &outputSignal;
    ss.A = A;
    ss.B = B;
    ss.C = C;
    ss.D = D;
    ss.dt = 0.001;

    // Instantiate system
    Eigen::VectorXf initialState(2);
    initialState << 0.05, 0;
    SimInterface::StateSpace block(ss, initialState,  0.f);

    // Instantiate Sink
    SimInterface::Sink sink(&outputSignal);

    // Test system
    SimInterface::SystemManager& systemManager = SimInterface::SystemManager::Get();
    for (float t = 0; t <= 5; t += 0.2) {

        // TODO: rather than step to t, step by dt
        systemManager.UpdateSystem(t);
    }

    return 0;
}
