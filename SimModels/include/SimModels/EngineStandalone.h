#ifndef FRAMEWORK_ENGINESTANDALONE_H
#define FRAMEWORK_ENGINESTANDALONE_H

#include <string>

#include "Eigen/Dense"
#include "SimFramework/Utilities.h"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

#include "SimModels/VehicleComponents.h"
#include "SimModels/Engine.h"


namespace Models {

    struct EngineBlocks
    {
        SimFramework::Input<float>* InputLoadBlock;
        SimFramework::Input<float>* InputThrottleBlock;
        SimFramework::Output<float>* OutputSpeedBlock;
    };


    class EngineStandalone : public SimFramework::System {
    public:
        EngineStandalone(std::string engineJSON="/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json", float initialSpeed=200.f, float J=1.f, float b=0.05);
        EngineBlocks Blocks();

    private:
        // Blocks
        SimFramework::Input<float> m_Throttle;
        SimFramework::Input<float> m_Load;
        SimFramework::Output<float> m_EngineSpeed;

        // Subsystems
        Engine m_Engine;
    };

} // namespace Models

#endif //FRAMEWORK_ENGINESTANDALONE_H
