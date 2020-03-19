#ifndef FRAMEWORK_ENGINESTANDALONE_H
#define FRAMEWORK_ENGINESTANDALONE_H

#include <string>

#include "Eigen/Dense"
#include "SimFramework/Utilities.h"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

#include "SimModels/VehicleComponents.h"


namespace Models {

    struct EngineBlocks
    {
        SimFramework::Input<float>* InputLoadBlock;
        SimFramework::Input<float>* InputThrottleBlock;
        SimFramework::Output<float>* OutputSpeedBlock;
    };


    class EngineStandalone : public SimFramework::System {
    public:
        EngineStandalone();
        void SetEngineParameters(std::string engineJSON="/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json", float J=1.f, float b=0.05);

        EngineBlocks Blocks();

    private:
        // Signals
        SimFramework::Signal<float> m_SThrottle;
        SimFramework::Signal<float> m_SLoadTorque;
        SimFramework::Signal<float> m_SEngineSpeed;

        // Blocks
        SimFramework::Input<float> m_BThrottle;
        SimFramework::Input<float> m_BLoad;
        SimFramework::Output<float> m_BEngineSpeed;

        // Subsystems
        Engine m_SysEngine;
    };

} // namespace Models

#endif //FRAMEWORK_ENGINESTANDALONE_H
