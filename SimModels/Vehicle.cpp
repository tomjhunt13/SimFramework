#include "SimModels/Vehicle.h"


namespace Models {

    Vehicle::Vehicle(float dt) : System(dt)
    {
        // Configure subsystems
        this->m_Controller.Configure(this->m_LockupClutch.OutSpeed2(), this->m_InThrottle.OutSignal(), this->m_Transmission.OutGearIndex());
        this->m_Engine.Configure(this->m_Controller.OutAugmentedThrottle(), this->m_Clutch.OutClutchTorque());
        this->m_Transmission.Configure(this->m_LockupClutch.OutSpeed2(), this->m_Tyre.OutTorque());
        this->m_VehicleDynamics.Configure(this->m_Tyre.OutForce(), this->m_Road.OutGradient(), this->m_InBrakePressure.OutSignal());

        // Configure model blocks
        this->m_Tyre.Configure(this->m_Transmission.OutTyreSpeed(), this->m_VehicleDynamics.OutVehicleVelocity());
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

        SimFramework::BlockList list = {{&(this->m_InThrottle),     &(this->m_InBrakePressure)},
                                        {},
                                        {&(this->m_Clutch), &(this->m_Tyre), &(this->m_Road)},
                                        {&(this->m_OutEngineSpeed), &(this->m_OutFuelFlowRate), &(this->m_OutFuelCumulative), &(this->m_OutWheelSpeed), &(this->m_OutLinearVelocity), &(this->m_OutDisplacement), &(this->m_OutCoordinates), &(this->m_OutGradient), &(this->m_OutCurrentGear)},
                                        {&(this->m_Controller), &(this->m_Engine),         &(this->m_Transmission), &(this->m_VehicleDynamics)}};
        this->RegisterBlocks(list);

    };

    void Vehicle::SetParameters(Models::VehicleParameters parameters) {

        this->m_Controller.SetParameters(parameters.GearshiftLag, parameters.ClutchStiffness);

        m_EngineTransmission.SetParameters(
                parameters.EngineJSON, parameters.EngineInitialSpeed,
                parameters.GearRatios, parameters.EngineViscousConstant, parameters.TransmissionViscousFriction, parameters.EngineInertia, parameters.TransmissionInertia, parameters.PeakClutchTorque)
        this->m_Engine.SetParameters(, parameters.EngineInitialSpeed, parameters.EngineInertia, parameters.EngineViscousConstant);
        this->m_Transmission.SetParameters(parameters.GearRatios, parameters.TransmissionInertia);
        this->m_VehicleDynamics.SetParameters(parameters.InitialPosition, parameters.InitialVelocity, parameters.Mass, parameters.Cd, parameters.A, parameters.rho, parameters.PeakBrakeForce, parameters.RollingResistance);
        this->m_Road.SetProfile(parameters.RoadJSON);

        this->m_Tyre.SetParameters(parameters.TyreRadius, parameters.Mass * 9.81 / 2.f, parameters.PeakTyreForceScale);

        this->SetLogOutputFile(parameters.LogOutputFile, parameters.LogFrequency);
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
                &(this->m_OutGradient)};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Vehicle::LogSignals()
    {
        return {{"Demand Throttle", this->m_InThrottle.OutSignal()},
                {"Brake Pressure", this->m_InBrakePressure.OutSignal()},
                {"Vehicle Position (x), Vehicle Position (y)", this->m_Road.OutPosition()},
                {"Road Gradient", this->m_Road.OutGradient()},
                {"Clutch Torque", this->m_Clutch.OutClutchTorque()},
                {"Displacement", this->m_VehicleDynamics.OutVehiclePosition()}};
    };


}; // namespace Models