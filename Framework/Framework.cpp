#include "SimFramework/Framework.h"
#include "SimFramework/Utilities.h"

namespace SimFramework {


    void Model::Initialise(float t_0)
    {
        if (!this->m_Configured)
        {
            this->Configure();
        }

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

        this->m_t_n = t_0;
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

    void Model::RegisterBlocks(BlockList& blocks)
    {
        this->m_Sources.insert(this->m_Sources.end(), blocks.Sources.begin(), blocks.Sources.end());
        this->m_DynamicSystems.insert(this->m_DynamicSystems.end(), blocks.DynamicSystems.begin(), blocks.DynamicSystems.end());
        this->m_Functions.insert(this->m_Functions.end(), blocks.Functions.begin(), blocks.Functions.end());
        this->m_Sinks.insert(this->m_Sinks.end(), blocks.Sinks.begin(), blocks.Sinks.end());

        for (Subsystem* subsys : blocks.Subsystems)
        {
            BlockList subsysBlocks = subsys->Blocks();
            this->RegisterBlocks(subsysBlocks);
        }
    };

    void Model::Configure()
    {
        this->m_Functions = Internal::SortFunctions(this->m_Functions);
        this->m_Configured = true;
    }



    namespace Internal {



        // Given a vector functions, return map of signals to the functions they drive
        std::map<SignalBase*, std::vector<Function*>> FunctionInputs(std::vector<Function*> functions)
        {

            std::map<SignalBase*, std::vector<Function*>> outputMap;

            // Iterate over functions
            for (Function* func : functions)
            {

                // Iterate over input signals
                for (SignalBase* sig : func->InputSignals())
                {

                    // If signal not already in map, create Function* vector
                    if (outputMap.count(sig) == 0)
                    {
                        outputMap.insert({sig, {}});
                    }

                    // Append signal to Function* vector
                    outputMap[sig].push_back(func);
                }
            }

            return outputMap;
        }

        std::vector<std::vector<int>> AdjacencyList(std::vector<Function*> functions)
        {

            // Map of Function* to index in list
            std::map<Function*, int> mapFuncIndex;
            for (int i = 0; i < functions.size(); i++)
            {
                mapFuncIndex.insert({functions[i], i});
            }

            // Map of signals to their driving functions
            std::map<SignalBase*, std::vector<Function*>> mapSigFunc = FunctionInputs(functions);

            std::vector<std::vector<int>> outputList(functions.size());

            // Iterate through functions
            for (int i=0; i < functions.size(); i++)
            {
                // Get outputs
                for (SignalBase* sig : functions[i]->OutputSignals())
                {
                    if (mapSigFunc.count(sig) > 0)
                    {
                        for (Function* drivenFunc : mapSigFunc[sig])
                        {
                            outputList[i].push_back(mapFuncIndex[drivenFunc]);
                        }
                    }
                }
            }

            return outputList;
        };

        void TopologicalSortRecursive(int index, std::vector<std::vector<int>>& adjacencyList, std::vector<bool>& visited, std::stack<int>& stack)
        {
            if (!visited[index])
            {
                // Append children
                for (int i : adjacencyList[index])
                {
                    TopologicalSortRecursive(i, adjacencyList, visited, stack);
                }

                // Add self to stack and mark as visited
                stack.push(index);
                visited[index] = true;
            }
        };

        std::vector<int> TopologicalSort(std::vector<std::vector<int>> adjacencyList)
        {

            std::stack<int> stack;
            std::vector<bool> visited(adjacencyList.size());

            for (int i = 0; i < adjacencyList.size(); i++)
            {
                TopologicalSortRecursive(i, adjacencyList, visited, stack);
            }

            // Unpack stack into vector
            std::vector<int> output(adjacencyList.size());

            for (int i = 0; i < adjacencyList.size(); i++)
            {
                output[i] = stack.top();
                stack.pop();
            }

            return output;
        };

        std::vector<Function*> SortFunctions(std::vector<Function*> functions)
        {
            std::vector<std::vector<int>> adjacencyList = AdjacencyList(functions);
            std::vector<int> sortedIndices = TopologicalSort(adjacencyList);

            std::vector<Function*> output(functions.size());
            for (int i = 0; i < functions.size(); i++)
            {
                output[i] = functions[sortedIndices[i]];
            }

            return output;
        };


    }; // namespace Internal



} // namespace Framework