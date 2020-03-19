#include "SimModels/Vehicle.h"


namespace Models {

    Vehicle::Vehicle() : System(0.001) {
        // Configure subsytems
        this->m_Engine.Configure(&(this->m_SThrottle), &(this->m_SClutchTorque), &(this->m_SEngineSpeed));
        this->m_Transmission.Configure(&(this->m_SClutchTorque), &(this->m_STyreTorque), &(this->m_SBrake),
                                       &(this->m_SClutchSpeed), &(this->m_STyreSpeed));
        this->m_VehicleDynamics.Configure(&(this->m_STyreForce), &(this->m_SCarPosition), &(this->m_SCarSpeed));


        this->m_Engine.SetEngineParameters();

        // Configure model blocks
        this->m_Clutch.Configure(&(this->m_SEngineSpeed), &(this->m_SClutchTorque));
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
                                        {&(this->m_Engine),         &(this->m_Transmission), &(this->m_VehicleDynamics)}};
        this->RegisterBlocks(list);

    };

    void Vehicle::ShiftUp()
    {
        this->m_Transmission.ShiftUp();
    };

    void Vehicle::ShiftDown()
    {
        this->m_Transmission.ShiftDown();
    };

    int Vehicle::CurrentGear() const
    {
        return this->CurrentGear();
    };

    const VehicleBlocks Vehicle::Blocks()
    {
        return {&(this->m_InThrottle), &(this->m_InBrakePressure), &(this->m_OutEngineSpeed), &(this->m_OutTyreSpeed), &(this->m_OutPosition), &(this->m_OutVelocity)};
    };
}; // namespace Models