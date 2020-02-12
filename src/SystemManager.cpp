#include "SystemManager.h"


namespace SimFramework {

    SystemManager & SystemManager::Get()
    {
        // Instantiate new SystemManager if one doesn't exist
        static SystemManager systemManager;
        return systemManager;
    }

    void SystemManager::RegisterBlock(Block* block)
    {
        SystemManager::Get().m_Blocks.push_back(block);
    };

    void SystemManager::UpdateSystem(float tMax)
    {

        SystemManager& systemManager = SystemManager::Get();

        // Read inputs
        for (auto i: systemManager.m_Blocks)
        {
            i->Read();
        }

        // Update
        for (auto i: systemManager.m_Blocks)
        {
            i->Update(tMax);
        }

        // Write outputs
        for (auto i: systemManager.m_Blocks)
        {
            i->Write();
        }
    }

} // namespace SimFramework