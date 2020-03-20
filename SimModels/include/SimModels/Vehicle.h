#ifndef FRAMEWORK_VEHICLE_H
#define FRAMEWORK_VEHICLE_H

#include <string>

#include "SimFramework/Utilities.h"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

#include "SimModels/VehicleComponents.h"

// TODO: Singleton with all vehicle parameters

namespace Models {

    struct VehicleParameters
    {
        // Engine
        std::string EngineJSON = "/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json";
        float EngineInitialSpeed = 200.f;
        float EngineInertia = 1.f;
        float EngineViscousConstant = 0.05;

        // Controller
        float GearshiftLag = 1.f;

        // Clutch
        float ClutchStiffness = 1000.f;

        // Transmission

        // Tyre

        // Vehicle
        float InitialPosition = 0.f;
        float InitialVelocity = 0.f;
        float Mass = 1000.f;
        float Cd = 0.3;
        float A = 2.5;
        float rho = 1.225;
    };

    struct VehicleBlocks
    {
        SimFramework::Input<float>* InThrottle;
        SimFramework::Input<float>* InBrakePressure;
        SimFramework::Output<float>* OutEngineSpeed;
        SimFramework::Output<float>* OutTyreSpeed;
        SimFramework::Output<float>* OutPosition;
        SimFramework::Output<float>* OutVelocity;
    };

    class Vehicle : public SimFramework::System
    {
    public:
        Vehicle(VehicleParameters parameters);
        void ShiftUp();
        void ShiftDown();
        int CurrentGear() const;

        const VehicleBlocks Blocks();


    private:
        // Signals
        SimFramework::Signal<float> m_SThrottle;
        SimFramework::Signal<float> m_SThrottleAugmented;
        SimFramework::Signal<float> m_SBrake;
        SimFramework::Signal<float> m_SEngineSpeed;
        SimFramework::Signal<float> m_SClutchSpeed;
        SimFramework::Signal<float> m_SClutchTorque;
        SimFramework::Signal<float> m_SClutchStiffness;
        SimFramework::Signal<float> m_STyreForce;
        SimFramework::Signal<float> m_STyreTorque;
        SimFramework::Signal<float> m_STyreSpeed;
        SimFramework::Signal<float> m_SCarPosition;
        SimFramework::Signal<float> m_SCarSpeed;

        // Blocks - IO
        SimFramework::Input<float> m_InThrottle;
        SimFramework::Input<float> m_InBrakePressure;
        SimFramework::Output<float> m_OutEngineSpeed;
        SimFramework::Output<float> m_OutTyreSpeed;
        SimFramework::Output<float> m_OutPosition;
        SimFramework::Output<float> m_OutVelocity;

        // Blocks - System
        Clutch m_Clutch;
        Tyre m_Tyre;

        // Subsystems
        VehicleController m_Controller;
        Engine m_Engine;
        Transmission m_Transmission;
        VehicleDynamics m_VehicleDynamics;
    };

}; // namespace Models

#endif //FRAMEWORK_VEHICLE_H
