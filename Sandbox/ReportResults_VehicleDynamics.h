#ifndef FRAMEWORK_REPORTRESULTS_VEHICLEDYNAMICS_H
#define FRAMEWORK_REPORTRESULTS_VEHICLEDYNAMICS_H

#include <string>
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"
#include "SimModels/VehicleDynamics.h"
#include "SimModels/VehicleComponents.h"


namespace Report {

    struct TestVehicleDynamicsParameters {
        float initialPosition;
        float initialVelocity;
        float mass;
        float Cd;
        float A;
        float rho;
        float rollingResistance;
        float PeakBrakeForce;
        float g=9.81;

        // Logging
        std::string LogOutputFile = "../Sandbox/Data/Results/ReportResults_Powertrain.csv";
        int LogFrequency = 1;
    };

    struct TestVehicleDynamicsBlocks {
        SimFramework::Input<float>* TyreForce;
        SimFramework::Input<float>* Gradient;
        SimFramework::Input<float>* BrakePedal;

        // Output
        SimFramework::Output<float>* Position;
        SimFramework::Output<float>* Velocity;
    };

    class TestVehicleDynamics : public SimFramework::System {
    public:
        TestVehicleDynamics(float dt = 0.001) : System(dt) {

            this->m_VehicleDynamics.Configure(this->m_TyreForce.OutSignal(), this->m_Gradient.OutSignal());
            this->m_Brake.Configure(this->m_VehicleDynamics.OutVehicleVelocity(), this->m_BrakePedal.OutSignal());

            // IO
            this->m_TyreForce.Configure(0.f);
            this->m_Gradient.Configure(0.f);
            this->m_Position.Configure(this->m_VehicleDynamics.OutVehiclePosition(), 0.f);
            this->m_Velocity.Configure(this->m_VehicleDynamics.OutVehicleVelocity(), 0.f);


            SimFramework::BlockList list = {{&(this->m_TyreForce), &(this->m_Gradient), &(this->m_BrakePedal)},
                                            {},
                                            {&(this->m_Brake)},
                                            {&(this->m_Position), &(this->m_Velocity)},
                                            {&(this->m_VehicleDynamics)}};
            this->RegisterBlocks(list);
        };

        void SetParameters(TestVehicleDynamicsParameters p) {
            this->m_VehicleDynamics.SetParameters(p.initialPosition, p.initialVelocity, p.mass, p.Cd, p.A, p.rho, p.rollingResistance, p.g);
            this->m_Brake.SetParameters(p.PeakBrakeForce);
            this->SetLogOutputFile(p.LogOutputFile, p.LogFrequency);
        }

        TestVehicleDynamicsBlocks Blocks() {
            return {&(this->m_TyreForce), &(this->m_Gradient), &(this->m_BrakePedal), &(this->m_Position), &(this->m_Velocity)};
        }


    private:
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override {
            return {{"Position", this->m_VehicleDynamics.OutVehiclePosition()},
                    {"Velocity", this->m_VehicleDynamics.OutVehicleVelocity()}};
        };

        // Input
        SimFramework::Input<float> m_TyreForce;
        SimFramework::Input<float> m_Gradient;
        SimFramework::Input<float> m_BrakePedal;

        // Output
        SimFramework::Output<float> m_Position;
        SimFramework::Output<float> m_Velocity;

        // Vehicle Dynamics
        Models::CoulombFriction m_Brake;
        Models::VehicleDynamics m_VehicleDynamics;
    };
}

#endif //FRAMEWORK_REPORTRESULTS_VEHICLEDYNAMICS_H
