#include "Framework.h"



namespace SimFramework {

    //----------------- Blocks
    Block::Block(e_BlockType blockType) : m_BlockType(blockType)
    {
        SystemManager::RegisterBlock(this);
    };

    void Block::RegisterInputSignal(Signal& inputSignal, std::string name)
    {
        this->m_InputSignals.insert({name, &inputSignal});
        this->m_InputSignalNames.push_back(name);
    };

    void Block::RegisterOutputSignal(Signal& inputSignal, std::string name)
    {
        this->m_InputSignals.insert({name, &inputSignal});
        this->m_InputSignalNames.push_back(name);
    };

    std::vector<Block*> Block::InputBlocks()
    {
        std::vector<Block*> outputList;

        for (auto i : this->m_InputSignalNames)
        {
            outputList.push_back(this->m_InputSignals[i]->InputBlock());
        }

        return outputList;
    };





    //----------------- SystemManager
    SystemManager & SystemManager::Get()
    {
        // Instantiate new SystemManager if one doesn't exist
        static SystemManager systemManager;
        return systemManager;
    }

    void SystemManager::RegisterBlock(Block* block)
    {
        switch (block->BlockType())
        {
            case Source : SystemManager::Get().m_Sources.push_back(block);
            case DynamicSystem : SystemManager::Get().m_DynamicSystems.push_back(block);
            case Function : SystemManager::Get().m_Functions.push_back(block);
            case Sink : SystemManager::Get().m_Sinks.push_back(block);
        }
    };

    void SystemManager::ConstructSystem()
    {
        this->OrderFunctions();
    };


    void SortTree(FunctionTree* node, std::vector<Block*>& blockList)
    {
        // If no children add to list
        if (node->children.empty())
        {
            blockList.push_back(node->functionBlock);
            return;
        }

        // Otherwise call sort tree on each child in turn
        for (FunctionTree* child : node->children)
        {
            SortTree(child, blockList);
        }

        // Then add current node to list
        blockList.push_back(node->functionBlock);
        return;
    }

    void SystemManager::OrderFunctions()
    {
        // Rearrange this->m_Functions into update order


        // Map of block pointers to trees
        std::map<Block*, FunctionTree> blockMap;


        // Iterate over each function block to fill map
        for (Block* func : this->m_Functions)
        {
            // Initialise tree for function block
            FunctionTree tree;
            tree.functionBlock = func;

            blockMap.insert({func, tree});
        }

        // Iterate over each function block again to fill tree with dependants
        for (Block* func : this->m_Functions)
        {
            for (Block* b : func->InputBlocks())
            {
                if (b->BlockType() == Function)
                {
                    blockMap[func].children.push_back(&blockMap[b]);
                    blockMap[b].root = false;
                }
            }
        }

        // Create ordered list
        std::vector<Block*> orderedList;
        for (Block* func : this->m_Functions)
        {
            if (blockMap[func].root) {

                // Depth first traversal from root
                std::vector<Block*> treeList;
                SortTree(&blockMap[func], treeList);

                // Add to ordered list
                orderedList.insert(orderedList.end(), treeList.begin(), treeList.end());
            }
        }

        this->m_Functions = orderedList;
    };


    void SystemManager::UpdateSystem(float tMax)
    {

        SystemManager& sys = SystemManager::Get();

        for (Block* func : sys.m_Functions)
        {
            func->Update(tMax);
        }

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