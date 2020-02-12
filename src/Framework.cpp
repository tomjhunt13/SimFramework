#include "Framework.h"


namespace SimFramework {

    //----------------- Blocks
    Block::Block(e_BlockType blockType) : m_BlockType(blockType)
    {
        this->m_ID = SystemManager::RegisterBlock(this);
    };



    //----------------- SystemManager
    SystemManager & SystemManager::Get()
    {
        // Instantiate new SystemManager if one doesn't exist
        static SystemManager systemManager;
        return systemManager;
    }

    unsigned int SystemManager::RegisterBlock(Block* block)
    {
        std::vector<Block*>& blockList = SystemManager::Get().m_Blocks;

        blockList.insert(blockList.end(), block);

        return blockList.size() - 1;
    };

    void SystemManager::UpdateSystem(float tMax)
    {

//        SystemManager& systemManager = SystemManager::Get();
//
//        // Read inputs
//        for (auto i: systemManager.m_Blocks)
//        {
//            i->Read();
//        }
//
//        // Update
//        for (auto i: systemManager.m_Blocks)
//        {
//            i->Update(tMax);
//        }
//
//        // Write outputs
//        for (auto i: systemManager.m_Blocks)
//        {
//            i->Write();
//        }
    }

} // namespace SimFramework