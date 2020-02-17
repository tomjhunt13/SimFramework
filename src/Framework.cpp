#include "Framework.h"


namespace SimFramework {

    //----------------- SystemManager
    struct FunctionTree {
        Block* functionBlock;
        std::vector<FunctionTree*> children;
        bool root = true;
    };

    SystemManager & SystemManager::Get()
    {
        // Instantiate new SystemManager if one doesn't exist
        static SystemManager systemManager;
        return systemManager;
    }

    void SystemManager::RegisterBlocks(std::vector<Block*> blocks)
    {
        SystemManager& sys = SystemManager::Get();
        sys.m_Blocks = blocks;
    };



    void SystemManager::UpdateSystem(float t_np1)
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
            i->Update(t_np1);
        }

        // Write outputs
        for (auto i: systemManager.m_Blocks)
        {
            i->Write();
        }
    };

} // namespace SimFramework