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

class DemoModel : public SimFramework::Model
{

public:
    DemoModel();


    const SimFramework::Input<float>& InputBlock();
    const SimFramework::Output<float>& OutputBlock();


private:

// Signals
    SimFramework::Signal<float> m_InputForce;
    SimFramework::Signal<float> m_SpringForce;
    SimFramework::Signal<float> m_DamperForce;
    SimFramework::Signal<float> m_SummedForce;

    SimFramework::Input<float> m_Input;
    SimFramework::ConstantBlock<float> m_ConstSpring;
    SimFramework::ConstantBlock<float> m_ConstDamper;
    SimFramework::SummingJunction<float> m_SumForces;
    SimFramework::Output<float> m_OutputBlock;

};

DemoModel::DemoModel()
{
    // Configure blocks
    this->m_Input.Configure(&(this->m_InputForce), 0.1);
    this->m_ConstSpring.Configure(&(this->m_SpringForce), 1.5);
    this->m_ConstDamper.Configure(&(this->m_DamperForce), 2.5);
    this->m_SumForces.Configure({&(this->m_InputForce), &(this->m_SpringForce), &(this->m_DamperForce)}, &(this->m_SummedForce), {1.f, 1.f, 1.f});
    this->m_OutputBlock.Configure(&m_SummedForce, 0.f);

    // Construct system
    this->RegisterBlocks(
            {&(this->m_Input), &(this->m_ConstSpring), &(this->m_ConstDamper)},
            {},
            {&(this->m_SumForces)},
            {&(this->m_OutputBlock)});
}


const SimFramework::Input<float>& DemoModel::InputBlock()
{
    return this->m_Input;
};

const SimFramework::Output<float>& DemoModel::OutputBlock()
{
    return this->m_OutputBlock;
};


class ExampleSystem : public SimFramework::Model
{
public:
    ExampleSystem()
    {
        std::string jsonFilePath = "/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json";
        SimFramework::Table3D engineTable = SimFramework::ReadTableJSON(jsonFilePath, "speed", "throttle", "torque");


        // Configure Blocks
        this->m_ThrottleBlock.Configure(&(this->m_Throttle), 100.f);
        this->m_Load.Configure(&(this->m_ConstLoad), 20);
        this->m_SummingJunction.Configure({&(this->m_EngOutTorque), &(this->m_ConstLoad)}, &(this->m_SummedLoad), {1.f, -1.f});
        this->m_Eng.Configure(engineTable, &(this->m_EngInSpeed), &(this->m_Throttle), &(this->m_EngOutTorque));

        float massValue = 0.1;
        Eigen::Matrix<float, 2, 2> A;
        A << 0.f, 1.f, 0.f, 0.f;
        Eigen::Matrix<float, 2, 1> B = {0.f, 1.f / massValue};
        Eigen::Matrix<float, 2, 2> C;
        C << 1.f, 0.f, 0.f, 1.f;
        Eigen::Matrix<float, 2, 1> D = {0.f, 0.f};
        Eigen::Vector2f initialValues = {0.f, 300.f};

        this->m_Mass.Configure(&(this->m_SummedLoad), &(this->m_MassStates), A, B, C, D, initialValues);
        this->m_Mask.Configure(&(this->m_MassStates), {&(this->m_EngInSpeed), &(this->m_EnginePos)}, {1, 0});
        this->m_Out.Configure(&(this->m_MassStates), &(this->m_SummedLoad));

        // Construct system
        this->RegisterBlocks(
                {&(this->m_ThrottleBlock), &(this->m_Load)},
                {&(this->m_Mass)},
                {&(this->m_Mask), &(this->m_Eng), &(this->m_SummingJunction)},
                {&(this->m_Out)});
    }


private:

    // Signals
    SimFramework::Signal<float> m_EnginePos;
    SimFramework::Signal<float> m_EngInSpeed;
    SimFramework::Signal<float> m_Throttle;
    SimFramework::Signal<float> m_EngOutTorque;
    SimFramework::Signal<float> m_ConstLoad;
    SimFramework::Signal<float> m_SummedLoad;
    SimFramework::Signal<Eigen::Vector2f> m_MassStates;

    // Blocks
    SimFramework::ConstantBlock<float> m_ThrottleBlock;
    SimFramework::ConstantBlock<float> m_Load;
    SimFramework::SummingJunction<float> m_SummingJunction;
    SimFramework::LookupTable2D m_Eng;
    SimFramework::StateSpace<float, Eigen::Vector2f, 1, 2, 2> m_Mass;
    SimFramework::Mask<Eigen::Vector2f, float> m_Mask;
    OutputBlock m_Out;


};


int main() {

    int a = sizeof(SimFramework::Block*);

    ExampleSystem sys;
    sys.Initialise(0);

    for (float t = 0; t <= 1; t += 0.01) {
        sys.Update(t);
    }

    return 0;
}
