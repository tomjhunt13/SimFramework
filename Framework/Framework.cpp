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

//        std::map<Function*, std::vector<SignalBase*>> FunctionInputs(std::vector<Function*> functions)
//        {
//            std::map<Function*, std::vector<SignalBase*>> outputMap;
//            for (Function* func : functions)
//            {
//                outputMap.insert({func, func->InputSignals()});
//            }
//
//            return outputMap;
//        }

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

        std::vector<FunctionTree> AssembleTree(std::vector<Function*> functions)
        {
            // Map of signals to their driving functions
            std::map<SignalBase*, Function*> mapSigFunc = FunctionOutputs(functions);

            // Map of function pointers to their trees
            std::map<Function*, FunctionTree> mapFuncTree;

            // Iterate over each function block and create corresponding tree
            for (Function* func : functions)
            {
                mapFuncTree.insert({func, {func}});
            }

            // Iterate over each function again to create tree structure
            for (Function* func : functions)
            {

                // Iterate over each driving signal of function
                for (SignalBase* signal : func->InputSignals())
                {

                    // If driving signal is driven by a function
                    if (mapSigFunc.count(signal) > 0)
                    {

                        // Add driving function of the driving signal as child of func function tree and mark root as false
                        mapFuncTree[func].children.push_back(&mapFuncTree[mapSigFunc[signal]]);
                        mapFuncTree[mapSigFunc[signal]].root = false;
                    }
                }
            }


            // Find roots of trees
            std::vector<FunctionTree> treeVector;

            for (Function* func : functions)
            {
                if (mapFuncTree[func].root)
                {
                    treeVector.push_back(mapFuncTree[func]);
                }
            }

            // If no root, algebraic loop!

            return treeVector;


//        FunctionTree AssembleTree(std::map<Function*, std::vector<SignalBase*>> inputMap, std::map<SignalBase*, Function*> outputMap)
//        {
//            // Map of block pointers to trees
//            std::map<Function*, FunctionTree> blockMap;
//
//            // Iterate over each function block and create tree
//            for (auto input : inputMap)
//            {
//                blockMap.insert({std::get<0>(input), {std::get<0>(input)}});
//            }
//
//            // Iterate over each function again and create tree structure
//            for (auto input : inputMap)
//            {
//
//                // Iterate over each child signal of function
//                for (auto signal : std::get<1>(input))
//                {
//
//                    // Get driving function block
//
//
//                }
//
//            }
//
//
//            // If no root, algebraic loop!
//
//
//
//            return {};

//            // Iterate over each function block again to fill tree with dependants
//            for (Block* func : this->m_Functions)
//            {
//                for (Block* b : func->InputBlocks())
//                {
//                    if (b->BlockType() == e_BlockType ::eFunction)
//                    {
//                        blockMap[func].children.push_back(&blockMap[b]);
//                        blockMap[b].root = false;
//                    }
//                }
//            }
//
//            // Create ordered list
//            std::vector<Block*> orderedList;
//            for (Block* func : this->m_Functions)
//            {
//
//                if (blockMap[func].root) {
//
//                    // Depth first traversal from root
//                    std::vector<Block*> treeList;
//                    SortTree(&blockMap[func], treeList);
//
//                    // Add to ordered list
//                    orderedList.insert(orderedList.end(), treeList.begin(), treeList.end());
//                }
//            }
//
//            this->m_Functions = orderedList;
        }

    }; // namespace Internal



} // namespace Framework