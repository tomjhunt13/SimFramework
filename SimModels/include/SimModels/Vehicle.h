#ifndef FRAMEWORK_VEHICLE_H
#define FRAMEWORK_VEHICLE_H

#include <string>

#include "Eigen/Dense"

#include "SimFramework/Utilities.h"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

#include "SimModels/VehicleComponents.h"
#include "SimModels/Engine.h"
#include "SimModels/VehicleDynamics.h"
#include "SimModels/VehicleController.h"
#include "SimModels/Transmission.h"
#include "SimModels/Road.h"


namespace Models {

    struct VehicleParameters
    {
        // Engine
        std::string EngineJSON;
        float EngineInitialSpeed = 200.f;
        float EngineInertia = 1.f;
        float EngineViscousConstant = 0.4;

        // Controller
        float GearshiftLag = 1.f;

        // Clutch
        float ClutchStiffness = 100.f;

        // Transmission
        std::vector<float> GearRatios = {11.f, 7.3, 5.5, 3.8, 2.5, 2.f};
        float TransmissionInertia = 2.f;

        // Brake
        float PeakBrakeForce = 300.f;

        // Tyre

        // Road
        std::string RoadJSON;

        // Vehicle
        float InitialPosition = 0.f;
        float InitialVelocity = 0.f;
        float Mass = 1000.f;
        float Cd = 0.3;
        float A = 2.5;
        float rho = 1.225;

        // Admin
        int LogFrequency = 5; // Number of complete updates until next log
        std::string LogOutputFile = "LogOut.csv";
    };


    struct VehicleBlocks
    {
        // Inputs
        SimFramework::Input<float>* InThrottle;
        SimFramework::Input<float>* InBrakePressure;

        // Engine
        SimFramework::Output<float>* OutEngineSpeed;
        SimFramework::Output<float>* OutFuelFlowRate;
        SimFramework::Output<float>* OutFuelCumulative;

        // Vehicle
        SimFramework::Output<float>* OutWheelSpeed;
        SimFramework::Output<float>* OutLinearVelocity;
        SimFramework::Output<float>* OutDisplacement;
        SimFramework::Output<Eigen::Vector2f>* OutCoordinates;

        // Transmission
        SimFramework::Output<int>* OutCurrentGear;

        // Road
        SimFramework::Output<float>* OutGradient; // Radians
    };

    class Vehicle : public SimFramework::System
    {
    public:
        Vehicle();

        const VehicleBlocks Blocks();
        void SetParameters(VehicleParameters parameters);

        void ShiftUp();
        void ShiftDown();

    private:
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;

        // Blocks - IO
        SimFramework::Input<float> m_InThrottle;
        SimFramework::Input<float> m_InBrakePressure;

        SimFramework::Output<float> m_OutEngineSpeed;
        SimFramework::Output<float> m_OutFuelFlowRate;
        SimFramework::Output<float> m_OutFuelCumulative;
        SimFramework::Output<float> m_OutWheelSpeed;
        SimFramework::Output<float> m_OutLinearVelocity;
        SimFramework::Output<float> m_OutDisplacement;
        SimFramework::Output<Eigen::Vector2f> m_OutCoordinates;
        SimFramework::Output<int> m_OutCurrentGear;
        SimFramework::Output<float> m_OutGradient;

        // Blocks - System
        Clutch m_Clutch;
        Tyre m_Tyre;
        Road m_Road;

        // Subsystems
        VehicleController m_Controller;
        Engine m_Engine;
        Transmission m_Transmission;
        VehicleDynamics m_VehicleDynamics;
    };

}; // namespace Models

#endif //FRAMEWORK_VEHICLE_H
