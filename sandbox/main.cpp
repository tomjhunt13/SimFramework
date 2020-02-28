#include <iostream>
#include <string>
#include <fstream>
#include "Eigen/Dense"

#include "Vehicle.h"

#include "Framework.h"
#include "Components.h"
#include "Utilities.h"


#include "Inertia1D.h"
#include "OutputBlock.h"
#include "Clutch.h"



int main() {



    std::string jsonFilePath = "/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json";

    SimFramework::Table3D engineTable = SimFramework::ReadTableJSON(jsonFilePath, "speed", "throttle", "torque");

    // Signals
    SimFramework::Signal<float> enginePos;
    SimFramework::Signal<float> engInSpeed;
    SimFramework::Signal<float> throttle;
    SimFramework::Signal<float> engOutTorque;
    SimFramework::Signal<float> constLoad;
    SimFramework::Signal<float> summedLoad;
    SimFramework::Signal<Eigen::Vector2f> massStates;

    // Blocks
    SimFramework::ConstantBlock<float> throttleBlock(&throttle, 100.f);
    SimFramework::ConstantBlock<float> load(&constLoad, 20);
    SimFramework::SummingJunction<float> summingJunction({&engOutTorque, &constLoad}, &summedLoad, {1.f, -1.f});
    SimFramework::LookupTable2D eng(engineTable, &engInSpeed, &throttle, &engOutTorque);
    Vehicle::Inertia1D mass(&summedLoad, &massStates, {0, 300});
    SimFramework::Mask<Eigen::Vector2f, float> mask(&massStates, {&engInSpeed, &enginePos}, {1, 0});
    OutputBlock out(&massStates, &summedLoad);

    // Construct system
    SimFramework::SystemManager& systemManager = SimFramework::SystemManager::Get();
    systemManager.RegisterBlocks({&throttleBlock, &load}, {&mass}, {&mask, &eng, &summingJunction}, {&out});
    systemManager.Initialise(0.f);

    // Iterate
    for (float t = 0; t <= 1; t += 0.01) {
        systemManager.UpdateSystem(t);
    }

    return 0;
}
