#include <iostream>

#include <vector>

#include "Eigen/Dense"

#include "SpringDamper1D.h"
#include "Mass1D.h"
#include "ConstantBlock.h"
#include "OutputBlock.h"

//#include "SystemManager.h"
//#include "Block.h"
#include "Signal.h"
//#include "StateSpace.h"
//#include "Sink.h"





int main() {

    // Create Signals
    SimInterface::Signal<Eigen::Vector2f> signal1({0.f, 0.f}); // ConstantBlock States
    SimInterface::Signal<Eigen::Vector2f> signal2({1.f, 0.f}); // Mass1D States
    SimInterface::Signal<float> signal3(0.f);                   // SpringDamper Force

    // Create blocks
    SimInterface::ConstantBlock <Eigen::Vector2f> cnstblk (signal1, {0.f, 0.f});
    SpringDamper1D sd(signal1, signal2, signal3);
    Mass1D mass (signal3, signal2);
    OutputBlock out(signal2, signal3);




    // Test system
    SimInterface::SystemManager& systemManager = SimInterface::SystemManager::Get();
    for (float t = 0; t <= 5; t += 0.01) {

//        signal2.Write({1, 2});

        systemManager.UpdateSystem(t);
    }

    return 0;
}
