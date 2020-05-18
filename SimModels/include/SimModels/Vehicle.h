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
#include "SimModels/Road.h"
#include "SimModels/Powertrain.h"
#include "SimModels/Wheel.h"


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
        float ClutchEngagementSpeed = 100.f;
        float PullawayClutchMinValue = 0.2;

        // Clutch
        float ClutchTorqueCapacity = 400.f;

        // Transmission
        std::vector<float> GearRatios = {1.f};
        float TransmissionInertia = 0.2;
        float TransmissionViscousFriction = 0.05;

        // Brake
        float PeakBrakeForce = 1000.f;

        // Tyre
        float TyreRadius = 0.2;
        float PeakTyreForceScale = 1.f;

        // Road
        std::string RoadJSON;

        // Vehicle
        float InitialPosition = 0.f;
        float InitialVelocity = 0.f;
        float Mass = 1000.f;
        float Cd = 0.3;
        float A = 2.5;
        float rho = 1.225;
        float RollingResistance = 0.015;

        // Stats
        EUnitSystem Units = EUnitSystem::e_Imperial;
        float FuelDensity = 0.7489;

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

        // Road
        SimFramework::Output<float>* OutGradient; // Radians

        // Powertrain
        SimFramework::Output<int>* OutCurrentGear;
        SimFramework::Output<int>* OutClutchLockState;

        // Efficiency Stats
        SimFramework::Output<float>* OutInstantaneousFuelEfficiency;
        SimFramework::Output<float>* OutAverageFuelEfficiency;
    };

    class Vehicle : public SimFramework::System
    {
    public:
        Vehicle(float dt=0.01);

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
        SimFramework::Output<float> m_OutInstantaneousFuelEfficiency;
        SimFramework::Output<float> m_OutAverageFuelEfficiency;
        SimFramework::Output<float> m_OutWheelSpeed;
        SimFramework::Output<float> m_OutLinearVelocity;
        SimFramework::Output<float> m_OutDisplacement;
        SimFramework::Output<Eigen::Vector2f> m_OutCoordinates;
        SimFramework::Output<int> m_OutCurrentGear;
        SimFramework::Output<float> m_OutGradient;
        SimFramework::Output<int> m_OutClutchLockState;

        // Blocks - System
        Wheel m_Wheel;
        UnitConversions m_UnitConversions;
        Road m_Road;

        // Subsystems
        VehicleController m_Controller;
        Powertrain m_Powertrain;
        Engine m_Engine;
        VehicleDynamics m_VehicleDynamics;
    };

}; // namespace Models

#endif //FRAMEWORK_VEHICLE_H
