#ifndef FRAMEWORK_SANDBOXLOCKUPCLUTCH_H
#define FRAMEWORK_SANDBOXLOCKUPCLUTCH_H

#include <iostream>

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

#include "SimModels/LockupClutch.h"

struct TestClutchLockupParameters {

    // Initial conditions
    float initSpeed1 = 0.f;
    float initSpeed2 = 0.f;

    // Physical parameters
    float b_1 = 1.f;
    float b_2 = 1.f;
    float I_1 = 1.f;
    float I_2 = 1.f;
    float PeakClutchTorque = 10.f;

    // Logging
    std::string LogOutputFile = "LogOut.csv";
    int LogFrequency = 1;
};

struct TestClutchLockupBlocks {
    SimFramework::Input<float>* Torque1;
    SimFramework::Input<float>* Torque2;
    SimFramework::Input<float>* Alpha;
    SimFramework::Output<float>* OutSpeed1;
    SimFramework::Output<float>* OutSpeed2;
};

class TestClutchLockup : public SimFramework::System
{
public:
    TestClutchLockup(float dt=0.01) : System(dt)
    {
        this->m_LockupClutch.Configure(this->m_Torque1.OutSignal(), this->m_Torque2.OutSignal(), this->m_Alpha.OutSignal());
        this->m_OutSpeed1.Configure(this->m_LockupClutch.OutSpeed1(), 0.f);
        this->m_OutSpeed2.Configure(this->m_LockupClutch.OutSpeed2(), 0.f);

        SimFramework::BlockList list = {{&(this->m_Torque1), &(this->m_Torque2), &(this->m_Alpha)},
                                        {},
                                        {},
                                        {&(this->m_OutSpeed1), &(this->m_OutSpeed2)},
                                        {&(this->m_LockupClutch)}};
        this->RegisterBlocks(list);
    };

    void SetParameters(TestClutchLockupParameters p)
    {
        this->m_LockupClutch.SetParameters(p.initSpeed1, p.initSpeed2, p.b_1, p.b_2, p.I_1, p.I_2, p.PeakClutchTorque);
        this->SetLogOutputFile(p.LogOutputFile, p.LogFrequency);
    }

    TestClutchLockupBlocks Blocks()
    {
        return {&(this->m_Torque1), &(this->m_Torque2), &(this->m_Alpha), &(this->m_OutSpeed1), &(this->m_OutSpeed2)};
    }


private:
    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override { return {}; };


    // Input
    SimFramework::Input<float> m_Torque1;
    SimFramework::Input<float> m_Torque2;
    SimFramework::Input<float> m_Alpha;

    // Output
    SimFramework::Output<float> m_OutSpeed1;
    SimFramework::Output<float> m_OutSpeed2;

    // Clutch
    Models::LockupClutch m_LockupClutch;
};


void SandboxLockupClutch() {


    // Set up system
    TestClutchLockup sys;
    TestClutchLockupBlocks blocks = sys.Blocks();

    TestClutchLockupParameters parameters;
    parameters.I_1 = 1.f;
    parameters.I_2 = 1.f;
    parameters.b_1 = 0.5;
    parameters.b_2 = 1.f;
    parameters.initSpeed1 = 0.f;
    parameters.initSpeed2 = 0.f;
    parameters.PeakClutchTorque = 10.f;

    sys.SetParameters(parameters);

    // Initial input values
    blocks.Torque1->WriteValue(10.f);
    blocks.Alpha->WriteValue(1.f);

    sys.Initialise(0);

    int counter = 0;

    for (float t = 0.f; t <= 300.f; t += 0.1) {


        if (counter == 600)
        {
            blocks.Alpha->WriteValue(0.f);
        }

        if (counter == 1200)
        {
            blocks.Alpha->WriteValue(1.f);
        }

        sys.Update(t);

        std::cout << "t: " << t << ", speed 1: " << blocks.OutSpeed1->ReadValue() << ", speed 2: " << blocks.OutSpeed2->ReadValue() << std::endl;

        counter ++;
    }
};

#endif //FRAMEWORK_SANDBOXLOCKUPCLUTCH_H
