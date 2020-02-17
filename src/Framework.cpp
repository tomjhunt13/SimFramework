#include "Framework.h"


namespace SimFramework {

    //----------------- Blocks
    Block::Block(e_BlockType blockType) : m_BlockType(blockType)
    {
        SystemManager::RegisterBlock(this);
    };

    void Block::RegisterInputSignal(Signal* inputSignal)
    {
        this->m_InputSignals.push_back(inputSignal);

    };

    void Block::RegisterOutputSignal(Signal* outputSignal)
    {
        outputSignal->SetInputBlock(this);
        this->m_OutputSignals.push_back(outputSignal);
    };


    std::vector<Block*> Block::InputBlocks()
    {
        std::vector<Block*> outputList;

        for (auto i : this->m_InputSignals)
        {
            outputList.push_back(i->GetInputBlock());
        }

        return outputList;
    };

    void  Block::Read()
    {
        for (int i=0; i < this->m_InputSignals.size(); i++)
        {
            this->m_InputCopy[i] = this->m_InputSignals[i]->Read();
        }
    }

    void  Block::Write()
    {
        for (int i=0; i < this->m_InputSignals.size(); i++)
        {
            this->m_OutputSignals[i]->Write(this->m_OutputCopy);
        }
    }


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

    void SystemManager::RegisterBlock(Block* block)
    {
        switch (block->BlockType())
        {
            case e_BlockType::eSource :
                SystemManager::Get().m_Sources.push_back(block);
                break;

            case e_BlockType::eDynamicSystem :
                SystemManager::Get().m_DynamicSystems.push_back(block);
                break;

            case e_BlockType::eFunction :
                SystemManager::Get().m_Functions.push_back(block);
                break;

            case e_BlockType::eSink :
                SystemManager::Get().m_Sinks.push_back(block);
                break;
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
                if (b->BlockType() == e_BlockType ::eFunction)
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


    void ReadBlocks(std::vector<Block*>* blocks)
    {
        // Read inputs
        for (Block* i: *blocks)
        {
            i->Read();
        }
    }

    void UpdateBlocks(std::vector<Block*>* blocks, float t_np1)
    {
        // Update
        for (Block* i: *blocks) {
            i->Update(t_np1);
        }
    }

    void WriteBlocks(std::vector<Block*>* blocks)
    {
        // Write outputs
        for (Block* i: *blocks)
        {
            i->Write();
        }
    }

    void SystemManager::UpdateSystem(float t_np1)
    {

        SystemManager& sys = SystemManager::Get();

        std::map<int, std::vector<Block*>*> updateOrder;
        updateOrder.insert({0, &sys.m_Sources});
        updateOrder.insert({1, &sys.m_DynamicSystems});
        updateOrder.insert({2, &sys.m_Functions});
        updateOrder.insert({3, &sys.m_Sinks});

        // Read, Update, Write
        for (int i = 0; i < 4; i++) {ReadBlocks(updateOrder[i]); };
        for (int i = 0; i < 4; i++) {UpdateBlocks(updateOrder[i], t_np1); };
        for (int i = 0; i < 4; i++) {WriteBlocks(updateOrder[i]); };
    }


    Eigen::VectorXf ForwardEuler::Step(DynamicSystem &system, float dt, float t_n, Eigen::VectorXf& x_n)
    {
        return x_n + dt * system.Gradient(t_n, x_n);
    }

} // namespace SimFramework