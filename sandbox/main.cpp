#include <iostream>
#include <string>
#include <fstream>

#include "nlohmann/json.hpp"

#include "../src/Framework.h"
#include "../src/Interpolation.h"






int main() {

    std::string jsonFilePath = "/Users/tom/Documents/University/Y4_S2/Data/engineExample.json";

    Engine eng(jsonFilePath);

    return 0;



//    SimFramework::Signal<Eigen::Vector2f> signal1; // ConstantBlock States
//    SimFramework::Signal<float> signal2; // SpringDamper Force
//    SimFramework::Signal<float> signal3; // Const Additional Force
//    SimFramework::Signal<float> signal4; // Sum of forces
//    SimFramework::Signal<Eigen::Vector2f> signal5; // Mass1D States
//
//    // Create blocks
//    SimFramework::ConstantBlock<Eigen::Vector2f> connection1 (&signal1, {0.f, 0.f});
//    SimFramework::ConstantBlock<float> constForce (&signal3, -10.f);
//    SpringDamper1D spring(&signal1, &signal5, &signal2);
//    SimFramework::SummingJunction<float> sum({&signal2, &signal3}, &signal4, {1.f, 1.f});
//    Mass1D mass (&signal4, &signal5, {0.1, 0.f});
//    OutputBlock out(&signal5, &signal4);
//
//    // Construct system
//    SimFramework::SystemManager& systemManager = SimFramework::SystemManager::Get();
//    systemManager.RegisterBlocks({&connection1, &constForce}, {&mass}, {&spring, &sum}, {&out});
//    systemManager.Initialise(0.f);
//
//    // Iterate
//    for (float t = 0; t <= 1000; t += 0.01) {
//        systemManager.UpdateSystem(t);
//    }

    return 0;
}
