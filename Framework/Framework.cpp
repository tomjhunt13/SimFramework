#include "SimFramework/Framework.h"

#include "SimFramework/Utilities.h"

namespace SimFramework {


    void Model::Initialise(float t_0)
    {
        for (auto i: this->m_Sources)
        {
            i->Initialise(t_0);
        }

        for (auto i: this->m_DynamicSystems)
        {
            i->Initialise(t_0);
        }

        for (auto i: this->m_Functions) {
            i->Update();
        }

        // Update sinks to t_np1
        for (auto i: this->m_Sinks) {
            i->Update(0.f);
        }
    }


    void Model::Update(float t_np1)
    {

        // Get steps
        std::vector<float> timeSteps = TimeSteps(this->m_t_n, t_np1, this->m_dtMax);

        for (float dt: timeSteps) {

            // All dynamic systems read input u at t_n
            for (auto i: this->m_DynamicSystems) {
                i->ReadInputs();
            }

            // Update dynamic systems and write output value at t_np1
            for (auto i: this->m_DynamicSystems) {
                i->Update(dt);
            }

            // Update functions in order, write out value at t_np1
            for (auto i: this->m_Functions) {
                i->Update();
            }

            // Update sinks to t_np1
            for (auto i: this->m_Sinks) {
                i->Update(dt);
            }

            // Update sources to t_np1
            for (auto i: this->m_Sources) {
                i->Update(dt);
            }
        }

        this->m_t_n = t_np1;
    };

    void Model::RegisterBlocks(
            std::vector<Source*> sources, std::vector<DynamicSystem*> dynamicSystems,
            std::vector<Function*> functions, std::vector<Sink*> sinks)
    {
        this->m_Sources = sources;
        this->m_DynamicSystems = dynamicSystems;
        this->m_Functions = functions;
        this->m_Sinks = sinks;
    };


    namespace Internal {

        std::map<SignalBase*, Function*> FunctionOutputs(std::vector<Function*> functions)
        {
            std::map<SignalBase*, Function*> outputMap;
            for (Function* func : functions)
            {
                for (SignalBase* sig : func->OutputSignals())
                {
                    outputMap.insert({sig, func});
                }
            }
            return outputMap;
        }

        void SortTree(FunctionTree* node, std::vector<Function*>& blockList)
        {
            // If no children add to list
            if (node->children.empty())
            {
                blockList.push_back(node->block);
                return;
            }

            // Otherwise call sort tree on each child in turn
            for (FunctionTree* child : node->children)
            {
                SortTree(child, blockList);
            }

            // Then add current node to list
            blockList.push_back(node->block);
            return;
        }

        std::vector<Function*> UnpackTree(FunctionTree& tree)
        {
            std::vector<Function*> outputVec;
            SortTree(&tree, outputVec);
            return outputVec;
        }

        std::vector<Function*> MergeOrderedFunctions(std::vector<std::vector<Function*>> functions)
        {
            return {};
        }



        std::vector<Function*> OrderFunctions(std::vector<Function*> functions)
        {

            // Map of function pointers to their trees
            std::vector<FunctionTree> functionTrees(functions.size());
            std::map<Function*, FunctionTree*> mapFuncTree;

            // Iterate over each function block and create corresponding tree
            for (int i = 0; i < functions.size(); i++)
            {
                functionTrees[i] = {functions[i]};
                mapFuncTree.insert({functions[i], &functionTrees[i]});
            }

            // Map of signals to their driving functions
            std::map<SignalBase*, Function*> mapSigFunc = FunctionOutputs(functions);

            // Iterate over each function again to create tree structure
            for (FunctionTree& tree : functionTrees)
            {

                // Iterate over each driving signal of function
                for (SignalBase* signal : tree.block->InputSignals())
                {
                    // If driving signal is driven by a function
                    if (mapSigFunc.count(signal) > 0)
                    {
                        // Add driving function of the driving signal as child of func function tree and mark root as false
                        tree.children.push_back(mapFuncTree[mapSigFunc[signal]]);
                        mapFuncTree[mapSigFunc[signal]]->root = false;
                    }
                }
            }

            // Unpack tree structures into ordered vector
            std::vector<Function*> orderedFunctions;
            for (FunctionTree& tree : functionTrees)
            {

                // TODO: If no root, algebraic loop!
                if (tree.root)
                {
                    std::vector<Function*> orderedTree = UnpackTree(tree);
                    orderedFunctions.insert(orderedFunctions.end(), orderedTree.begin(), orderedTree.end());
                };
            }

            return orderedFunctions;
        }

    }; // namespace Internal



} // namespace Framework