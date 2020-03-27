#ifndef SIMFRAMEWORK_FRAMEWORK_H
#define SIMFRAMEWORK_FRAMEWORK_H

#include <string>
#include <vector>
#include <map>
#include <stack>

#include "Utilities.h"

namespace SimFramework {


    //----------------- Signal
    class SignalBase
    {
    public:
        SignalBase(std::string name = "Signal");
        virtual const std::string ValueToString() const = 0;


    private:
        std::string m_Name;
    };

    template <typename SignalType>
    class Signal : public SignalBase
    {
    public:
        Signal(std::string name = "Signal") : SignalBase(name) {};

        SignalType Read() const
        {
            return this->m_Value;
        };

        void Write(const SignalType& value)
        {
            this->m_Value = value;
        };

        const std::string ValueToString() const override
        {
            return ToString(this->m_Value);
        };

    private:
        SignalType m_Value;
    };


    //----------------- Block
    class Block
    {
    public:
        Block(std::string name = "Block");

        virtual ~Block() {};

        virtual std::vector<const SignalBase*> InputSignals() const = 0;
        virtual std::vector<const SignalBase*> OutputSignals() const = 0;

    private:
        std::string m_Name;
    };

    class Source : public Block
    {
    public:
        Source(std::string name = "Source");

        virtual ~Source() {};
        virtual void Initialise(float t_0) = 0;     // Updates internal states AND writes initial values
        virtual void Update(float dt) = 0;

    };

    class DynamicSystem : public Block
    {
    public:
        DynamicSystem(std::string name = "Dynamic System");

        virtual ~DynamicSystem() {};
        virtual void ReadInputs() = 0;
        virtual void Initialise(float t_0) = 0;     // Set initial states AND write initial output
        virtual void Update(float dt) = 0;
    };

    class Function : public Block
    {
    public:
        Function(std::string name = "Function");

        virtual ~Function() {};
        virtual void Update() = 0;
    };

    class Sink : public Block
    {
    public:
        Sink(std::string name = "Sink");

        virtual ~Sink() {};
        virtual void Update(float dt) = 0;
    };


    class Subsystem;

    struct BlockList
    {
        std::vector<Source*> Sources;
        std::vector<DynamicSystem*> DynamicSystems;
        std::vector<Function*> Functions;
        std::vector<Sink*> Sinks;
        std::vector<Subsystem*> Subsystems;
    };

    class Subsystem
    {
    public:
        virtual BlockList Blocks() = 0;
        virtual std::vector<std::pair<std::string, const SignalBase*>> LogSignals() { return {}; };
    };

    class System {
    public:
        System(float dtMax=0.1);
        void Initialise(float t_0);
        void Update(float t_np1);
        void SetLogOutputFile(std::string outputCSVPath);

    protected:
        void RegisterBlocks(BlockList& blocks);
        virtual std::vector<std::pair<std::string, const SignalBase*>> LogSignals() { return {}; };

    private:
        void Configure();
        void LogOneSignal(std::string name, const SignalBase* signal);
        void UpdateLoggedSignals(float t);

        std::vector<Source*> m_Sources;
        std::vector<DynamicSystem*> m_DynamicSystems;
        std::vector<Function*> m_Functions;
        std::vector<Sink*> m_Sinks;

        std::vector<const SignalBase*> m_LogSignals;
        std::vector<std::string> m_LogNames;
        CSVWriter m_CSV;

        bool m_Configured;
        float m_dtMax;
        float m_t_n;
    };


    //-------- Dynamic system and integration
    template <typename DerivativeType>
    class Integrable {
    public:
        virtual DerivativeType Derivative(float t, const DerivativeType& x) = 0;
    };

    class ForwardEuler {
    public:
        template<typename DerivativeType>
        static DerivativeType Step(Integrable<DerivativeType>* block, float dt, float t_n, const DerivativeType& x_n)
        {
            return x_n + dt * block->Derivative(t_n, x_n);
        };
    };

    class RK4 {
    public:
        template<typename DerivativeType>
        static DerivativeType Step(Integrable<DerivativeType>* block, float dt, float t_n, const DerivativeType& x_n)
        {
            DerivativeType k1 = dt * block->Derivative(t_n, x_n);
            DerivativeType k2 = dt * block->Derivative(t_n + dt / 2.f, x_n + k1 / 2.f);
            DerivativeType k3 = dt * block->Derivative(t_n + dt / 2.f, x_n + k2 / 2.f);
            DerivativeType k4 = dt * block->Derivative(t_n + dt, x_n + k3);

            return x_n + (1.f / 6.f) * (k1 + 2 * k2 + 2 * k3 + k4);
        };
    };


    namespace Internal {

        std::map<const SignalBase*, std::vector<Function*>> FunctionInputs(std::vector<Function*> functions);
        std::vector<std::vector<int>> AdjacencyList(std::vector<Function*> functions);
        std::vector<int> TopologicalSort(std::vector<std::vector<int>> adjacencyList);
        std::vector<Function*> SortFunctions(std::vector<Function*> functions);

    }; // namespace Internal




} // namespace Framework

#endif //SIMFRAMEWORK_FRAMEWORK_H
