#include "SimModels/Vehicle.h"


namespace Models {

    Vehicle::Vehicle() : System(0.0025)
    {
        // Configure subsystems
        this->m_Controller.Configure(&(this->m_SThrottle), &(this->m_SClutchSpeed), &(this->m_SThrottleAugmented), &(this->m_SClutchStiffness));
        this->m_Engine.Configure(&(this->m_SThrottleAugmented), &(this->m_SClutchTorque), &(this->m_SEngineSpeed));
        this->m_Transmission.Configure(&(this->m_SClutchTorque), &(this->m_STyreTorque), &(this->m_SBrake),
                                       &(this->m_SClutchSpeed), &(this->m_STyreSpeed));
        this->m_VehicleDynamics.Configure(&(this->m_STyreForce), &(this->m_SCarPosition), &(this->m_SCarSpeed));

        // Configure model blocks
        this->m_Clutch.Configure(&(this->m_SEngineSpeed), &(this->m_SClutchSpeed), &(this->m_SClutchStiffness), &(this->m_SClutchTorque));
        this->m_Tyre.Configure(&(this->m_STyreSpeed), &(this->m_SCarSpeed), &(this->m_STyreForce),
                               &(this->m_STyreTorque));

        // Configure IO blocks
        this->m_InThrottle.Configure(&(this->m_SThrottle), 0.f);
        this->m_InBrakePressure.Configure(&(this->m_SBrake), 0.f);
        this->m_OutEngineSpeed.Configure(&(this->m_SEngineSpeed), 0.f);
        this->m_OutTyreSpeed.Configure(&(this->m_STyreSpeed), 0.f);
        this->m_OutPosition.Configure(&(this->m_SCarPosition), 0.f);
        this->m_OutVelocity.Configure(&(this->m_SCarSpeed), 0.f);

        SimFramework::BlockList list = {{&(this->m_InThrottle),     &(this->m_InBrakePressure)},
                                        {},
                                        {&(this->m_Clutch), &(this->m_Tyre)},
                                        {&(this->m_OutEngineSpeed), &(this->m_OutTyreSpeed), &(this->m_OutPosition), &(this->m_OutVelocity)},
                                        {&(this->m_Controller), &(this->m_Engine),         &(this->m_Transmission), &(this->m_VehicleDynamics)}};
        this->RegisterBlocks(list);
    };

    void Vehicle::SetParameters(Models::VehicleParameters parameters) {

        this->m_Controller.SetParameters(parameters.GearshiftLag, parameters.ClutchStiffness);
        this->m_Engine.SetParameters(parameters.EngineJSON, parameters.EngineInitialSpeed, parameters.EngineInertia, parameters.EngineViscousConstant);
        this->m_Transmission.SetParameters(parameters.GearRatios, parameters.TransmissionInertia, parameters.BrakeFrictionCoefficient, parameters.BrakeRadius, parameters.BrakeCylinderDiameter, parameters.MaxBrakePressure, parameters.BrakeCylindersPerWheel);
        this->m_VehicleDynamics.SetParameters(parameters.InitialPosition, parameters.InitialVelocity, parameters.Mass, parameters.Cd, parameters.A, parameters.rho);
    };



    void Vehicle::ShiftUp()
    {
        if (this->m_Transmission.ShiftUp())
        {
            this->m_Controller.Trigger();
        }
    };

    void Vehicle::ShiftDown()
    {
        if (this->m_Transmission.ShiftDown())
        {
            this->m_Controller.Trigger();
        };
    };

    int Vehicle::CurrentGear() const
    {
        return this->m_Transmission.CurrentGear();
    };

    const VehicleBlocks Vehicle::Blocks()
    {
        return {&(this->m_InThrottle), &(this->m_InBrakePressure), &(this->m_OutEngineSpeed), &(this->m_OutTyreSpeed), &(this->m_OutPosition), &(this->m_OutVelocity)};
    };
}; // namespace Models