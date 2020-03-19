#include "SimModels/VehicleDynamicsStandalone.h"

namespace Models {

    VehicleDynamicsStandalone::VehicleDynamicsStandalone() : System(0.1)
    {
        // Configure blocks
        this->m_TyreForce.Configure(&(this->m_STyre), 0.f);
        this->m_VehiclePosition.Configure(&(this->m_SPos), 0.f);
        this->m_VehicleSpeed.Configure(&(this->m_SVel), 0.f);
        this->m_SysVehicleDynamics.Configure(&(this->m_STyre), &(this->m_SPos), &(this->m_SVel));

        SimFramework::BlockList list = {
                {&(this->m_TyreForce)},
                {},
                {},
                {&(this->m_VehiclePosition), &(this->m_VehicleSpeed)},
                {&(this->m_SysVehicleDynamics)}};
        this->RegisterBlocks(list);
    };

    VehicleDynamicsBlocks VehicleDynamicsStandalone::Blocks()
    {
        return {&(this->m_TyreForce), &(this->m_VehiclePosition), &(this->m_VehicleSpeed)};
    };


}; // namespace Models;