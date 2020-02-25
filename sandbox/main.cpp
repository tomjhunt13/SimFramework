#include <iostream>
#include <string>
#include <fstream>
#include "Eigen/Dense"

#include "Vehicle.h"

#include "Framework.h"
#include "Components.h"

#include "Mass1D.h"
#include "Engine.h"
#include "OutputBlock.h"
#include "Clutch.h"

class Mask : public SimFramework::Block
{
public:
    Mask(SimFramework::Signal<Eigen::Vector2f>* inVec, SimFramework::Signal<float>* outFloat) : m_Vec(inVec), m_Float(outFloat) {}

    void Read() override
    {
        this->m_InCopyVec = this->m_Vec->Read();
    };

    void Write() override
    {
        this->m_Float->Write(this->m_OutCopyFloat);
    };

    void Update(float t_np1) override
    {
        this->m_OutCopyFloat = this->m_InCopyVec[1] * 0.104719755119660;
    };

    void Init(float t_0) override {};

private:
    SimFramework::Signal<Eigen::Vector2f>* m_Vec;
    SimFramework::Signal<float>* m_Float;

    Eigen::Vector2f m_InCopyVec;
    float m_OutCopyFloat;

};




int main() {



    std::string jsonFilePath = "/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json";


    // Signals
    SimFramework::Signal<float> engInSpeed;
    SimFramework::Signal<float> throttle;
    SimFramework::Signal<float> engOutTorque;
    SimFramework::Signal<float> constLoad;
    SimFramework::Signal<float> summedLoad;
    SimFramework::Signal<Eigen::Vector2f> massStates;

    // Blocks
    SimFramework::ConstantBlock<float> load(&constLoad, 50);
    SimFramework::SummingJunction<float> summingJunction({&engOutTorque, &constLoad}, &summedLoad, {1.f, -1.f});
    Engine eng(&engInSpeed, &throttle, &engOutTorque, jsonFilePath);
    Mass1D mass(&summedLoad, &massStates, {0, 3000});
    OutputBlock out(&massStates, &summedLoad);


    throttle.Write(100);


    // Construct system
    SimFramework::SystemManager& systemManager = SimFramework::SystemManager::Get();
    systemManager.RegisterBlocks({&load}, {&mass}, {&eng, &summingJunction}, {&out});
    systemManager.Initialise(0.f);

    // Iterate
    for (float t = 0; t <= 100; t += 0.01) {
        systemManager.UpdateSystem(t);
    }

    return 0;



//    Framework::Signal<Eigen::Vector2f> signal1; // ConstantBlock States
//    Framework::Signal<float> signal2; // SpringDamper Force
//    Framework::Signal<float> signal3; // Const Additional Force
//    Framework::Signal<float> signal4; // Sum of forces
//    Framework::Signal<Eigen::Vector2f> signal5; // Mass1D States
//
//    // Create blocks
//    Framework::ConstantBlock<Eigen::Vector2f> connection1 (&signal1, {0.f, 0.f});
//    Framework::ConstantBlock<float> constForce (&signal3, -10.f);
//    SpringDamper1D spring(&signal1, &signal5, &signal2);
//    Framework::SummingJunction<float> sum({&signal2, &signal3}, &signal4, {1.f, 1.f});
//    Mass1D mass (&signal4, &signal5, {0.1, 0.f});
//    OutputBlock out(&signal5, &signal4);
//
//    // Construct system
//    Framework::SystemManager& systemManager = Framework::SystemManager::Get();
//    systemManager.RegisterBlocks({&connection1, &constForce}, {&mass}, {&spring, &sum}, {&out});
//    systemManager.Initialise(0.f);
//
//    // Iterate
//    for (float t = 0; t <= 1000; t += 0.01) {
//        systemManager.UpdateSystem(t);
//    }

    return 0;
}
