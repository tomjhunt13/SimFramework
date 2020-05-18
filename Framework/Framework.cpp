#include "SimFramework/Framework.h"

namespace SimFramework {


    // Names for signals and blocks
    SignalBase::SignalBase(std::string name) : m_Name(name) {};
    Block::Block(std::string name) : m_Name(name) {};
    Source::Source(std::string name) : Block(name) {};
    DynamicSystem::DynamicSystem(std::string name) : Block(name) {};
    Function::Function(std::string name) : Block(name) {};
    Sink::Sink(std::string name) : Block(name) {};


    System::System(float dtMax) : m_Configured(false), m_LogFrequency(1), m_dtMax(dtMax)
    {
        this->m_LogNames.push_back("t");
        this->m_LogSignals.push_back(nullptr);
    }

    void System::Initialise(float t_0)
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
            i->Update(t_0);
        }

        this->m_t_n = t_0;

        // Update CSV Writer
        for (auto pair : this->LogSignals())
        {
            this->LogOneSignal(pair.first, pair.second);
        }
        this->m_CSV.SetHeader(this->m_LogNames);
        this->m_UpdateCounter = 0;
    }


    void System::Update(float t_np1)
    {

        // Get steps
        std::vector<float> timeSteps = Internal::TimeSteps(this->m_t_n, t_np1, this->m_dtMax);

        for (float dt: timeSteps) {

            // All dynamic systems read input u at t_n
            for (auto i: this->m_DynamicSystems) {
                i->ReadInputs();
            }

            // Update dynamic systems and write output value at t_np1
            for (auto i: this->m_DynamicSystems) {
                i->Update(dt);
            }

            // Update sources to t_np1
            for (auto i: this->m_Sources) {
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
        }

        this->m_t_n = t_np1;

        if (this->m_UpdateCounter % this->m_LogFrequency == 0)
        {
            this->UpdateLoggedSignals(this->m_t_n);
        }
        this->m_UpdateCounter += 1;
    };

    void System::RegisterBlocks(BlockList& blocks)
    {
        this->m_Sources.insert(this->m_Sources.end(), blocks.Sources.begin(), blocks.Sources.end());
        this->m_DynamicSystems.insert(this->m_DynamicSystems.end(), blocks.DynamicSystems.begin(), blocks.DynamicSystems.end());
        this->m_Functions.insert(this->m_Functions.end(), blocks.Functions.begin(), blocks.Functions.end());
        this->m_Sinks.insert(this->m_Sinks.end(), blocks.Sinks.begin(), blocks.Sinks.end());

        for (Subsystem* subsys : blocks.Subsystems)
        {
            // Register blocks
            BlockList subsysBlocks = subsys->Blocks();
            this->RegisterBlocks(subsysBlocks);

            // Log signals
            for (auto pair : subsys->LogSignals())
            {
                this->LogOneSignal(pair.first, pair.second);
            }
        }
    };

    void System::SetLogOutputFile(std::string outputCSVPath, int updateFrequency)
    {
        this->m_CSV.SetOutputFilepath(outputCSVPath);
        this->m_LogFrequency = updateFrequency;
    };

    void System::LogOneSignal(std::string name, const SignalBase* signal)
    {
        this->m_LogNames.push_back(name);
        this->m_LogSignals.push_back(signal);
    };


    void System::Configure()
    {
        this->m_Functions = Internal::SortFunctions(this->m_Functions);
        this->m_Configured = true;
    }

    void System::UpdateLoggedSignals(float t)
    {
        std::vector<std::string> elements;
        elements.push_back(ToString(t));

        for (auto signal : this->m_LogSignals)
        {
            if (signal)
            {
                elements.push_back(signal->ValueToString());
            }
        }

        this->m_CSV.AppendRow(elements);
    };


    namespace Internal {

        std::vector<float> TimeSteps(float tMin, float tMax, float dtMax) {

            float tRange = tMax - tMin;

            std::vector<float> timesteps;

            if (tRange > 0.f) {

                if (dtMax >= tRange) {
                    timesteps.push_back(tRange);
                } else {
                    int nt = std::ceilf(tRange / dtMax);
                    float dtNew = tRange / (float) nt;

                    for (int i = 0; i < nt; i++) {
                        timesteps.push_back(dtNew);
                    }
                }
            }

            return timesteps;
        };



        // Given a vector functions, return map of signals to the functions they drive
        std::map<const SignalBase*, std::vector<Function*>> FunctionInputs(std::vector<Function*> functions)
        {

            std::map<const SignalBase*, std::vector<Function*>> outputMap;

            // Iterate over functions
            for (Function* func : functions)
            {

                // Iterate over input signals
                for (const SignalBase* sig : func->InputSignals())
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
            std::map<const SignalBase*, std::vector<Function*>> mapSigFunc = FunctionInputs(functions);

            std::vector<std::vector<int>> outputList(functions.size());

            // Iterate through functions
            for (int i=0; i < functions.size(); i++)
            {
                // Get outputs
                for (const SignalBase* sig : functions[i]->OutputSignals())
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