#include <iostream>
#include <string>
#include <fstream>
#include "Eigen/Dense"



#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"
#include "SimFramework/Utilities.h"

#include "Vehicle/Vehicle.h"
#include "Vehicle/OutputBlock.h"
#include "Vehicle/Clutch.h"



int main() {



    std::string jsonFilePath = "/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json";

    SimFramework::Table3D engineTable = SimFramework::ReadTableJSON(jsonFilePath, "speed", "throttle", "torque");

    SimFramework::Signal<float> m_InputForce;

    SimFramework::Input<float>* m_InputBlock = new SimFramework::Input<float>(&m_InputForce, 0.f);

    delete m_InputBlock;

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

    Eigen::Matrix<float, 2, 2> A;
    A << 0.f, 1.f, 0.f, 0.f;

    Eigen::Matrix<float, 2, 1> B = {0.f, 1.f / massValue};

    Eigen::Matrix<float, 2, 2> C;
    C << 1.f, 0.f, 0.f, 1.f;

    Eigen::Matrix<float, 2, 1> D = {0.f, 0.f};

    Eigen::Vector2f initialValues(2);
    initialValues(0) = 0.f;
    initialValues(1) = 300.f;

    std::cout << "A: " << A << ", B: " << B << ", C: " << C << ",  D: " << D << std::endl;

    SimFramework::StateSpace<float, Eigen::Vector2f, 1, 2, 2> mass(&summedLoad, &massStates, A, B, C, D, initialValues);

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
