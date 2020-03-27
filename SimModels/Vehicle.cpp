#include "SimModels/Vehicle.h"


namespace Models {

    Vehicle::Vehicle() : System(0.0025)
    {
        // Configure subsystems
        this->m_Controller.Configure(this->m_InThrottle.OutSignal(), this->m_Transmission.OutClutchSpeed());
        this->m_Engine.Configure(this->m_Controller.OutAugmentedThrottle(), this->m_Clutch.OutClutchTorque());
        this->m_Transmission.Configure(this->m_Clutch.OutClutchTorque(), this->m_Tyre.OutTorque(), this->m_InBrakePressure.OutSignal());
        this->m_VehicleDynamics.Configure(this->m_Tyre.OutForce());

        // Configure model blocks
        this->m_Clutch.Configure(this->m_Engine.OutEngineSpeed(), this->m_Transmission.OutClutchSpeed(), this->m_Controller.OutClutchStiffness());
        this->m_Tyre.Configure(this->m_Transmission.OutTyreSpeed(), this->m_VehicleDynamics.OutVehicleVelocity());

        // Configure IO blocks
        this->m_InThrottle.Configure(0.f);
        this->m_InBrakePressure.Configure(0.f);
        this->m_OutEngineSpeed.Configure(this->m_Engine.OutEngineSpeed(), 0.f);
        this->m_OutTyreSpeed.Configure(this->m_Transmission.OutTyreSpeed(), 0.f);
        this->m_OutPosition.Configure(this->m_VehicleDynamics.OutVehiclePosition(), 0.f);
        this->m_OutVelocity.Configure(this->m_VehicleDynamics.OutVehicleVelocity(), 0.f);

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