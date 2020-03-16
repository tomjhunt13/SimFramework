#ifndef FRAMEWORK_VEHICLE_H
#define FRAMEWORK_VEHICLE_H

#include "SimFramework/Utilities.h"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

#include "SimModels/VehicleComponents.h"

namespace Models {

    struct VehicleBlocks
    {

    };

    class Vehicle : public SimFramework::Model
    {
    public:
        Vehicle()
        {
            this->m_Engine.Configure(&(this->m_SThrottle), &(this->m_SClutchTorque), &(this->m_SEngineSpeed));
            this->m_Clutch.Configure(&(this->m_SEngineSpeed), &(this->m_SClutchSpeed), &(this->m_SClutchTorque));
            this->m_Transmission.Configure(&(this->m_SClutchTorque), &(this->m_STyreTorque), &(this->m_SClutchSpeed), &(this->m_STyreSpeed));

            SimFramework::BlockList list = {{&(this->m_InThrottle)},
                                            {},
                                            {&(this->m_Clutch)},
                                            {&(this->m_OutEngineSpeed), &(this->m_OutTyreSpeed)},
                                            {&(this->m_Engine), &(this->m_Transmission)}};
            this->RegisterBlocks(list);
        }

    private:
        // Signals
        SimFramework::Signal<float> m_SThrottle;
        SimFramework::Signal<float> m_SEngineSpeed;
        SimFramework::Signal<float> m_SClutchSpeed;
        SimFramework::Signal<float> m_SClutchTorque;
        SimFramework::Signal<float> m_STyreTorque;
        SimFramework::Signal<float> m_STyreSpeed;

        // Blocks - IO
        SimFramework::Input<float> m_InThrottle;
        SimFramework::Output<float> m_OutEngineSpeed;
        SimFramework::Output<float> m_OutTyreSpeed;

        // Blocks - Model
        Clutch m_Clutch;

        // Subsystems
        Engine m_Engine;
        Transmission m_Transmission;
    };

}; // namespace Models

#endif //FRAMEWORK_VEHICLE_H
