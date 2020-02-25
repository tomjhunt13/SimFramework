#include <iostream>
#include <string>
#include <fstream>
#include "Eigen/Dense"

#include "Vehicle.h"

#include "Framework.h"
#include "Components.h"
#include "Utilities.h"


#include "Mass1D.h"
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

    SimFramework::Table3D engineTable = SimFramework::ReadTableJSON(jsonFilePath, "speed", "throttle", "torque");

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
    SimFramework::LookupTable2D eng(engineTable, &engInSpeed, &throttle, &engOutTorque);
    Mass1D mass(&summedLoad, &massStates, {0, 300});
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
}
