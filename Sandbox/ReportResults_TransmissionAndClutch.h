#ifndef FRAMEWORK_REPORTRESULTS_TRANSMISSIONANDCLUTCH_H
#define FRAMEWORK_REPORTRESULTS_TRANSMISSIONANDCLUTCH_H

#include <string>
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"
#include "SimModels/Powertrain.h"

namespace Report {

    struct TestClutchLockupParameters {

        // Initial conditions
        float initSpeed1 = 0.f;
        float initSpeed2 = 0.f;

        // Physical parameters
        float G = 1.f;
        float b_1 = 1.f;
        float b_2 = 1.f;
        float I_1 = 1.f;
        float I_2 = 1.f;
        float PeakClutchTorque = 10.f;

        // Logging
        std::string LogOutputFile = "../Sandbox/Data/Results/ReportResults_Powertrain.csv";
        int LogFrequency = 1;
    };

    struct TestClutchLockupBlocks {
        SimFramework::Input<float> *Torque1;
        SimFramework::Input<float> *Torque2;
        SimFramework::Input<float> *Alpha;
        SimFramework::Output<float> *OutSpeed1;
        SimFramework::Output<float> *OutSpeed2;
        SimFramework::Output<int> *OutEngagement;
    };

    class TestClutchLockup : public SimFramework::System {
    public:
        TestClutchLockup(float dt = 0.001) : System(dt) {
            this->m_Powertrain.Configure(this->m_Torque1.OutSignal(), this->m_Torque2.OutSignal(),
                                         this->m_Alpha.OutSignal());

            // IO
            this->m_Alpha.Configure(0.f);
            this->m_Torque1.Configure(0.f);
            this->m_Torque2.Configure(0.f);
            this->m_OutSpeed1.Configure(this->m_Powertrain.OutEngineSpeed(), 0.f);
            this->m_OutSpeed2.Configure(this->m_Powertrain.OutWheelSpeed(), 0.f);
            this->m_Engagement.Configure(this->m_Powertrain.OutEngagement(), 0);


            SimFramework::BlockList list = {{&(this->m_Torque1),   &(this->m_Torque2),   &(this->m_Alpha)},
                                            {},
                                            {},
                                            {&(this->m_OutSpeed1), &(this->m_OutSpeed2), &(this->m_Engagement)},
                                            {&(this->m_Powertrain)}};
            this->RegisterBlocks(list);
        };

        void SetParameters(TestClutchLockupParameters p) {
            this->m_Powertrain.SetParameters({p.G}, p.initSpeed1, p.initSpeed2, p.b_1, p.b_2, p.I_1, p.I_2,
                                             p.PeakClutchTorque);
            this->SetLogOutputFile(p.LogOutputFile, p.LogFrequency);
        }

        TestClutchLockupBlocks Blocks() {
            return {&(this->m_Torque1), &(this->m_Torque2), &(this->m_Alpha), &(this->m_OutSpeed1),
                    &(this->m_OutSpeed2), &(this->m_Engagement)};
        }


    private:
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override {
            return {{"Engine Speed",       this->m_Powertrain.OutEngineSpeed()},
                    {"Clutch Speed", this->m_Powertrain.OutClutchSpeed()},
                    {"Wheel Speed", this->m_Powertrain.OutWheelSpeed()},
                    {"Alpha",              this->m_Alpha.OutSignal()}};
        };


        // Input
        SimFramework::Input<float> m_Torque1;
        SimFramework::Input<float> m_Torque2;
        SimFramework::Input<float> m_Alpha;

        // Output
        SimFramework::Output<float> m_OutSpeed1;
        SimFramework::Output<float> m_OutSpeed2;
        SimFramework::Output<int> m_Engagement;

        // Clutch
        Models::Powertrain m_Powertrain;
    };

}

void ReportResults_TransmissionAndClutch()
{

    Report::TestClutchLockup sys;

    Report::TestClutchLockupParameters parameters;
    parameters.G = 2.f;
    parameters.I_1 = 1.f;
    parameters.I_2 = 1.f;
    parameters.b_1 = 0.05;
    parameters.b_2 = 0.05;
    parameters.initSpeed1 = 100.f;
    parameters.initSpeed2 = 300.f;
    parameters.PeakClutchTorque = 10.f;

    sys.SetParameters(parameters);

    Report::TestClutchLockupBlocks blocks = sys.Blocks();


    float alpha = 0.f;
    float T_e = 0.f;
    float T_w = 0.f;

    blocks.Alpha->WriteValue(alpha);
    blocks.Torque1->WriteValue(T_e);
    blocks.Torque2->WriteValue(T_w);

    sys.Initialise(0);

    int counter = 0;

    for (float t = 0.f; t <= 40.f; t += 0.1) {


        if (counter == 50)
        {
            alpha = 1.f;
            T_e = 0.f;
            T_w = 0.f;

        }

        if (counter == 200)
        {
            alpha = 1.f;
            T_e = 10.f;
            T_w = 0.f;
        }

        if (counter == 300)
        {
            alpha = 0.f;
            T_e = 10.f;
            T_w = 0.f;
        }

        blocks.Alpha->WriteValue(alpha);
        blocks.Torque1->WriteValue(T_e);
        blocks.Torque2->WriteValue(T_w);


        sys.Update(t);
        counter ++;
    }

}



#endif //FRAMEWORK_REPORTRESULTS_TRANSMISSIONANDCLUTCH_H
