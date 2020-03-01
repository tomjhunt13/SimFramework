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

    float massValue = 0.1;
    Eigen::MatrixXf A(2, 2);
    A(0, 0) = 0.f;
    A(0, 1) = 1.f;
    A(1, 0) = 0.f;
    A(1, 1) = 0.f;

    Eigen::MatrixXf B(2, 1);
    B(0, 0) = 0.f;
    B(1, 0) = 1.f / massValue;

    Eigen::MatrixXf C(2, 2);
    C(0, 0) = 1.f;
    C(0, 1) = 0.f;
    C(1, 0) = 0.f;
    C(1, 1) = 1.f;

    Eigen::MatrixXf D(2, 1);
    D(0, 0) = 0.f;
    D(1, 0) = 0.f;

    Eigen::Vector2f initialValues(2);
    initialValues(0) = 0.f;
    initialValues(1) = 300.f;


    SimFramework::StateSpace<float, Eigen::Vector2f> mass(&summedLoad, &massStates, A, B, C, D, initialValues);

//    Vehicle::Inertia1D mass(&summedLoad, &massStates, {0, 300});
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
