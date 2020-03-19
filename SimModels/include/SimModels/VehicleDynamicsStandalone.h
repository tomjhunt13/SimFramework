#ifndef FRAMEWORK_VEHICLEDYNAMICSSTANDALONE_H
#define FRAMEWORK_VEHICLEDYNAMICSSTANDALONE_H


#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

#include "SimModels/VehicleComponents.h"

namespace Models {

    struct VehicleDynamicsBlocks
    {
        SimFramework::Input<float>* InTyreForce;
        SimFramework::Output<float>* OutVehiclePosition;
        SimFramework::Output<float>* OutVehicleSpeed;
    };


    class VehicleDynamicsStandalone : public SimFramework::System {
    public:
        VehicleDynamicsStandalone();
        VehicleDynamicsBlocks Blocks();

    private:
        // Signals
        SimFramework::Signal<float> m_STyre;
        SimFramework::Signal<float> m_SPos;
        SimFramework::Signal<float> m_SVel;

        // Blocks
        SimFramework::Input<float> m_TyreForce;
        SimFramework::Output<float> m_VehiclePosition;
        SimFramework::Output<float> m_VehicleSpeed;
        Models::VehicleDynamics m_SysVehicleDynamics;

    };

}; // namespace Models


#endif //FRAMEWORK_VEHICLEDYNAMICSSTANDALONE_H
