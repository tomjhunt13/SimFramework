#include <iostream>

#include <vector>

#include "Eigen/Dense"

#include "SpringDamper1D.h"
#include "Mass1D.h"
#include "ConstantBlock.h"
#include "OutputBlock.h"
#include "SummingJunction.h"


int main() {

    // Create Signals
    SimFramework::Signal<Eigen::Vector2f> signal1({0.f, 0.f}); // ConstantBlock States
    SimFramework::Signal<Eigen::Vector2f> signal2({1.f, 0.f}); // Mass1D States
    SimFramework::Signal<float> signal3(0.f);                   // SpringDamper Force
    SimFramework::Signal<float> signal4(0.f);
    SimFramework::Signal<float> signal5(0.f);

    // Create blocks
    SimFramework::ConstantBlock <Eigen::Vector2f> cnstblk (signal1, {0.f, 0.f});
    Mass1D mass (signal5, signal2);

    // Forces
    SimFramework::ConstantBlock<float> constWeight (signal4, 0.5);
    SpringDamper1D springDamper(signal1, signal2, signal3);

    // Summong junction
    std::vector<SimFramework::Signal<float>*> inputSignals = {&signal3, &signal4};
    std::vector<float> weights = {-1.f, 1.f};
    SimFramework::SummingJunction<float>(inputSignals, signal5, weights);


    OutputBlock out(signal2, signal3);




    // Test system
    SimFramework::SystemManager& systemManager = SimFramework::SystemManager::Get();
    for (float t = 0; t <= 5; t += 0.01) {

//        signal2.Write({1, 2});

        systemManager.UpdateSystem(t);
    }

    return 0;
}
