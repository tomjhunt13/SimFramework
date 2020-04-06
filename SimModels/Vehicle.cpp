#include "SimModels/Vehicle.h"


namespace Models {

    Vehicle::Vehicle() : System(0.01)
    {
        // Configure subsystems
        this->m_Controller.Configure(this->m_InThrottle.OutSignal(), this->m_Transmission.OutClutchSpeed());
        this->m_Engine.Configure(this->m_Controller.OutAugmentedThrottle(), this->m_Clutch.OutClutchTorque());
        this->m_Transmission.Configure(this->m_Clutch.OutClutchTorque(), this->m_Tyre.OutTorque());
        this->m_VehicleDynamics.Configure(this->m_Tyre.OutForce(), this->m_Road.OutGradient());

        // Configure model blocks
        this->m_Clutch.Configure(this->m_Engine.OutEngineSpeed(), this->m_Transmission.OutClutchSpeed(), this->m_Controller.OutClutchStiffness());
        this->m_Tyre.Configure(this->m_Transmission.OutTyreSpeed(), this->m_VehicleDynamics.OutVehicleVelocity());
        this->m_Road.Configure(this->m_VehicleDynamics.OutVehiclePosition());

        // Configure IO blocks
        this->m_InThrottle.Configure(0.f);
        this->m_InBrakePressure.Configure(0.f);
        this->m_OutEngineSpeed.Configure(this->m_Engine.OutEngineSpeed(), 0.f);
        this->m_OutTyreSpeed.Configure(this->m_Transmission.OutTyreSpeed(), 0.f);
        this->m_OutPosition.Configure(this->m_VehicleDynamics.OutVehiclePosition(), 0.f);
        this->m_OutVelocity.Configure(this->m_VehicleDynamics.OutVehicleVelocity(), 0.f);
        this->m_OutCoordinates.Configure(this->m_Road.OutPosition(), Eigen::Vector2f::Zero());
        this->m_OutGradient.Configure(this->m_Road.OutGradient(), 0.f);
        this->m_OutCurrentGear.Configure(this->m_Transmission.OutGearIndex(), 0);

        SimFramework::BlockList list = {{&(this->m_InThrottle),     &(this->m_InBrakePressure)},
                                        {},
                                        {&(this->m_Clutch), &(this->m_Tyre), &(this->m_Road)},
                                        {&(this->m_OutEngineSpeed), &(this->m_OutTyreSpeed), &(this->m_OutPosition), &(this->m_OutVelocity), &(this->m_OutCoordinates), &(this->m_OutGradient), &(this->m_OutCurrentGear)},
                                        {&(this->m_Controller), &(this->m_Engine),         &(this->m_Transmission), &(this->m_VehicleDynamics)}};
        this->RegisterBlocks(list);

    };

    void Vehicle::SetParameters(Models::VehicleParameters parameters) {

        this->m_Controller.SetParameters(parameters.GearshiftLag, parameters.ClutchStiffness);
        this->m_Engine.SetParameters(parameters.EngineJSON, parameters.EngineInitialSpeed, parameters.EngineInertia, parameters.EngineViscousConstant);
        this->m_Transmission.SetParameters(parameters.GearRatios, parameters.TransmissionInertia);
        this->m_VehicleDynamics.SetParameters(parameters.InitialPosition, parameters.InitialVelocity, parameters.Mass, parameters.Cd, parameters.A, parameters.rho);
        this->m_Road.SetProfile(parameters.RoadJSON);

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
                &(this->m_OutTyreSpeed),
                &(this->m_OutPosition),
                &(this->m_OutVelocity),
                &(this->m_OutCurrentGear),
                &(this->m_OutCoordinates),
                &(this->m_OutGradient)};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Vehicle::LogSignals()
    {
        return {{"Demand Throttle", this->m_InThrottle.OutSignal()},
                {"Brake Pressure", this->m_InBrakePressure.OutSignal()},
                {"Vehicle Position (x), Vehicle Position (y)", this->m_Road.OutPosition()},
                {"Road Gradient", this->m_Road.OutGradient()}};
    };


}; // namespace Models