#include "SimModels/EngineStandalone.h"


namespace Models {

    EngineStandalone::EngineStandalone(std::string engineJSON, float initialSpeed, float J, float b) : System(0.001)
    {
        this->m_Engine.SetParameters(engineJSON, initialSpeed, J, b);

        // Configure blocks
        this->m_Throttle.Configure(0.f);
        this->m_Load.Configure(0.f);
        this->m_EngineSpeed.Configure(this->m_Engine.OutEngineSpeed(), 0.f);

        // Configure engine
        this->m_Engine.Configure(this->m_Throttle.OutSignal(), this->m_Load.OutSignal());

        // Construct system
        SimFramework::BlockList list = {{&(this->m_Throttle), &(this->m_Load)},
                                        {},
                                        {},
                                        {&(this->m_EngineSpeed)},
                                        {&(this->m_Engine)}};
        this->RegisterBlocks(list);
    }

    EngineBlocks EngineStandalone::Blocks()
    {
        return {&(this->m_Load), &(this->m_Throttle), &(this->m_EngineSpeed)};
    };


}; // namespace Models