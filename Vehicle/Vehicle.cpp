#include "Vehicle/Vehicle.h"

#include <iostream>
#include <string>


#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"
#include "Vehicle/Clutch.h"



namespace Vehicle {


//    struct VehicleComponents
//    {
////        // Signals
////        SimFramework::Signal<float>* ThrottlePosition; // [%]
////        SimFramework::Signal<float>* EngineSpeed; // [rad/s]
////        SimFramework::Signal<float>* EngineTorque; // [Nm]
////        SimFramework::Signal<Eigen::Vector2f> massStates; // 2 element vector {[rad], [rad/s]}
////
////        SimFramework::Signal<float> constLoad;
////        SimFramework::Signal<float> summedLoad;
//
//
//        // Engine
//        SimFramework::LookupTable2D* engine;
//        Inertia1D* engineInertia;
//        Clutch* clutchTorque;
//
//
//
//
//
//
//    };
//
//    Vehicle::Vehicle()
//    {
//        std::string jsonFilePath = "/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json";
//
//        SimFramework::Table3D engineTable = SimFramework::ReadTableJSON(jsonFilePath, "speed", "throttle", "torque");
//
//        // Signals
//        SimFramework::Signal<float> engInSpeed;
//        SimFramework::Signal<float> throttle;
//        SimFramework::Signal<float> engOutTorque;
//        SimFramework::Signal<float> constLoad;
//        SimFramework::Signal<float> summedLoad;
//        SimFramework::Signal<Eigen::Vector2f> massStates;
//
//        // Blocks
//        SimFramework::ConstantBlock<float> load(&constLoad, 50);
//        SimFramework::SummingJunction<float> summingJunction({&engOutTorque, &constLoad}, &summedLoad, {1.f, -1.f});
//        SimFramework::LookupTable2D eng(engineTable, &engInSpeed, &throttle, &engOutTorque);
//        Vehicle::Inertia1D mass(&summedLoad, &massStates, {0, 300});
//        OutputBlock out(&massStates, &summedLoad);
//
//
//        throttle.Write(100);
//
//
//        // Construct system
//        SimFramework::SystemManager& systemManager = SimFramework::SystemManager::Get();
//        systemManager.RegisterBlocks({&load}, {&mass}, {&eng, &summingJunction}, {&out});
//        systemManager.Initialise(0.f);
//
//        SimFramework::LookupTable2D engine = new SimFramework::LookupTable2D(engineTable, &engInSpeed, &throttle, &engOutTorque);
//        Inertia1D engineInertia;
//        Clutch clutchTorque;
//
//        this->components = new VehicleComponents;
//
//        this->components->engine = ;
//    }
//
//    Vehicle::~Vehicle()
//    {
//        delete this->components;
//    }
//
//    void Vehicle::Print() {
//        std::cout << this->components->ts.a << this->components->ts.b << std::endl;
//    }

}