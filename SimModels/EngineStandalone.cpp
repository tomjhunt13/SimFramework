#include "SimModels/EngineStandalone.h"


namespace Models {

    EngineStandalone::EngineStandalone(std::string engineJSON, float initialSpeed, float J, float b) : System(0.001)
    {
        this->m_SysEngine.SetParameters(engineJSON, initialSpeed, J, b);

        // Configure blocks
//        this->m_BThrottle.Configure(&(this->m_SThrottle), 0.f);
//        this->m_BLoad.Configure(&(this->m_SLoadTorque), 0.f);
        this->m_BEngineSpeed.Configure(&(this->m_SEngineSpeed), 0.f);

        // Configure engine
        this->m_SysEngine.Configure(&(this->m_SThrottle), &(this->m_SLoadTorque), &(this->m_SEngineSpeed));

        // Construct system
        SimFramework::BlockList list = {{&(this->m_BThrottle), &(this->m_BLoad)},
                                        {},
                                        {},
                                        {&(this->m_BEngineSpeed)},
                                        {&(this->m_SysEngine)}};
        this->RegisterBlocks(list);
    }

    EngineBlocks EngineStandalone::Blocks()
    {
        return {&(this->m_BLoad), &(this->m_BThrottle), &(this->m_BEngineSpeed)};
    };


}; // namespace Models