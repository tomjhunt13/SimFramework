#include "SimModels/Vehicle.h"


namespace Models {

    Vehicle::Vehicle(float dt) : System(dt)
    {
        // Configure subsystems
        this->m_LockupClutch.Configure(this->m_Engine.OutEngineTorque(), this->m_Transmission.OutClutchTorque(), this->m_Controller.OutClutchStiffness());
        this->m_Controller.Configure(this->m_LockupClutch.OutSpeed2(), this->m_InThrottle.OutSignal(), this->m_Transmission.OutGearIndex());
        this->m_Engine.Configure(this->m_Controller.OutAugmentedThrottle(), this->m_LockupClutch.OutSpeed1());
        this->m_Transmission.Configure(this->m_LockupClutch.OutSpeed2(), this->m_Wheel.OutTorque());
        this->m_VehicleDynamics.Configure(this->m_Wheel.OutForce(), this->m_Road.OutGradient());
        this->m_Wheel.Configure(this->m_InBrakePressure.OutSignal(), this->m_Transmission.OutWheelSpeed(), this->m_VehicleDynamics.OutVehicleVelocity());

        // Configure model blocks
        this->m_Road.Configure(this->m_VehicleDynamics.OutVehiclePosition());

        // Configure IO blocks
        this->m_InThrottle.Configure(0.f);
        this->m_InBrakePressure.Configure(0.f);
        this->m_OutEngineSpeed.Configure(this->m_LockupClutch.OutSpeed1(), 0.f);
        this->m_OutFuelFlowRate.Configure(this->m_Engine.OutFuelRate(), 0.f);
        this->m_OutFuelCumulative.Configure(this->m_Engine.OutFuelCumulative(), 0.f);
        this->m_OutWheelSpeed.Configure(this->m_Transmission.OutWheelSpeed(), 0.f);
        this->m_OutLinearVelocity.Configure(this->m_VehicleDynamics.OutVehicleVelocity(), 0.f);
        this->m_OutCoordinates.Configure(this->m_Road.OutPosition(), Eigen::Vector2f::Zero());
        this->m_OutDisplacement.Configure(this->m_VehicleDynamics.OutVehiclePosition(), 0.f);
        this->m_OutGradient.Configure(this->m_Road.OutGradient(), 0.f);
        this->m_OutCurrentGear.Configure(this->m_Transmission.OutGearIndex(), 0);
        this->m_OutClutchLockState.Configure(this->m_LockupClutch.OutEngagement(), 0);

        SimFramework::BlockList list = {{&(this->m_InThrottle),     &(this->m_InBrakePressure)},
                                        {},
                                        {&(this->m_Road)},
                                        {&(this->m_OutEngineSpeed), &(this->m_OutFuelFlowRate), &(this->m_OutFuelCumulative), &(this->m_OutWheelSpeed), &(this->m_OutLinearVelocity), &(this->m_OutDisplacement), &(this->m_OutCoordinates), &(this->m_OutGradient), &(this->m_OutCurrentGear), &(this->m_OutClutchLockState)},
                                        {&(this->m_Controller), &(this->m_Engine), &(this->m_Transmission), &(this->m_VehicleDynamics), &(this->m_LockupClutch), &(this->m_Wheel)}};
        this->RegisterBlocks(list);

    };

    void Vehicle::SetParameters(Models::VehicleParameters parameters) {

        this->m_Controller.SetParameters(parameters.GearshiftLag, parameters.ClutchEngagementSpeed, parameters.PullawayClutchMinValue);

        this->m_LockupClutch.SetParameters(parameters.EngineInitialSpeed, 0.f, parameters.EngineViscousConstant, parameters.TransmissionViscousFriction, parameters.EngineInertia, parameters.TransmissionInertia, parameters.ClutchMaxNormalForce, parameters.ClutchTorqueCapacity);
        this->m_Engine.SetParameters(parameters.EngineJSON);
        this->m_Transmission.SetParameters(parameters.GearRatios);
        this->m_VehicleDynamics.SetParameters(parameters.InitialPosition, parameters.InitialVelocity, parameters.Mass, parameters.Cd, parameters.A, parameters.rho, parameters.RollingResistance);
        this->m_Road.SetProfile(parameters.RoadJSON);

        this->m_Wheel.SetParameters(parameters.PeakBrakeForce, parameters.TyreRadius);

        this->SetLogOutputFile(parameters.LogOutputFile, parameters.LogFrequency);
    };



    void Vehicle::ShiftUp()
    {
        // TODO: if up from neutral don't trigger
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

    const VehicleBlocks Vehicle::Blocks()
    {
        return {&(this->m_InThrottle),
                &(this->m_InBrakePressure),
                &(this->m_OutEngineSpeed),
                &(this->m_OutFuelFlowRate),
                &(this->m_OutFuelCumulative),
                &(this->m_OutLinearVelocity),
                &(this->m_OutLinearVelocity),
                &(this->m_OutDisplacement),
                &(this->m_OutCoordinates),
                &(this->m_OutCurrentGear),
                &(this->m_OutGradient),
                &(this->m_OutClutchLockState)
        };
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Vehicle::LogSignals()
    {
        return {{"Demand Throttle", this->m_InThrottle.OutSignal()},
                {"Brake Pressure", this->m_InBrakePressure.OutSignal()},
                {"Vehicle Position (x), Vehicle Position (y)", this->m_Road.OutPosition()},
                {"Road Gradient", this->m_Road.OutGradient()},
                {"Wheel Speed", this->m_Transmission.OutWheelSpeed()},
                {"Tyre Torque On Clutch", this->m_Transmission.OutClutchTorque()},
                {"Tyre Torque", this->m_Wheel.OutTorque()},
                {"Displacement", this->m_VehicleDynamics.OutVehiclePosition()}};
    };


}; // namespace Models