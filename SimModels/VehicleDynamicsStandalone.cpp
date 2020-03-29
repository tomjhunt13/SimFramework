#include "SimModels/VehicleDynamicsStandalone.h"

namespace Models {

    VehicleDynamicsStandalone::VehicleDynamicsStandalone(VehicleDynamicsParams parameters) : System(0.1)
    {
        this->m_SysVehicleDynamics.SetParameters(parameters.InitialPosition, parameters.InitialVelocity, parameters.Mass, parameters.Cd, parameters.A, parameters.rho);

        // Configure blocks
        this->m_TyreForce.Configure(0.f);
        this->m_Gradient.Configure(0.f);
        this->m_VehiclePosition.Configure(this->m_SysVehicleDynamics.OutVehiclePosition(), 0.f);
        this->m_VehicleSpeed.Configure(this->m_SysVehicleDynamics.OutVehicleVelocity(), 0.f);
        this->m_SysVehicleDynamics.Configure(this->m_TyreForce.OutSignal(), this->m_Gradient.OutSignal());

        SimFramework::BlockList list = {
                {&(this->m_TyreForce), &(this->m_Gradient)},
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