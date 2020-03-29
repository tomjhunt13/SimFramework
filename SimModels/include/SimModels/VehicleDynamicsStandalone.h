#ifndef FRAMEWORK_VEHICLEDYNAMICSSTANDALONE_H
#define FRAMEWORK_VEHICLEDYNAMICSSTANDALONE_H


#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

#include "SimModels/VehicleComponents.h"

namespace Models {

    struct VehicleDynamicsParams
    {
        float InitialPosition = 0.f;
        float InitialVelocity = 0.f;
        float Mass = 1000.f;
        float Cd = 0.3;
        float A = 2.5;
        float rho = 1.225;
    };

    struct VehicleDynamicsBlocks
    {
        SimFramework::Input<float>* InTyreForce;
        SimFramework::Output<float>* OutVehiclePosition;
        SimFramework::Output<float>* OutVehicleSpeed;
    };

    class VehicleDynamicsStandalone : public SimFramework::System {
    public:
        VehicleDynamicsStandalone(VehicleDynamicsParams parameters);
        VehicleDynamicsBlocks Blocks();

    private:
        // Blocks
        SimFramework::Input<float> m_TyreForce;
        SimFramework::Output<float> m_VehiclePosition;
        SimFramework::Output<float> m_VehicleSpeed;
        Models::VehicleDynamics m_SysVehicleDynamics;
        SimFramework::ConstantBlock<float> m_Gradient;
    };

}; // namespace Models


#endif //FRAMEWORK_VEHICLEDYNAMICSSTANDALONE_H
