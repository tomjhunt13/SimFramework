#ifndef SIMFRAMEWORK_FRAMEWORK_H
#define SIMFRAMEWORK_FRAMEWORK_H

#include <vector>
#include <map>
#include <stack>

namespace SimFramework {


    //----------------- Signal
    class SignalBase {};

    template <typename SignalType>
    class Signal : public SignalBase
    {
    public:
        Signal() {};

        SignalType Read() const { return this->m_Value; };
        void Write(SignalType value) { this->m_Value = value; };

    private:
        SignalType m_Value;
    };


    //----------------- Block
    class Block
    {
    public:
        virtual ~Block() {};

        virtual std::vector<SignalBase*> InputSignals() = 0;
        virtual std::vector<SignalBase*> OutputSignals() = 0;
    };

    class Source : public Block
    {
    public:
        virtual ~Source() {};
        virtual void Initialise(float t_0) = 0;     // Updates internal states AND writes initial values
        virtual void Update(float dt) = 0;

    };

    class DynamicSystem : public Block
    {
    public:
        virtual ~DynamicSystem() {};
        virtual void ReadInputs() = 0;
        virtual void Initialise(float t_0) = 0;     // Only updates internal states
        virtual void Update(float dt) = 0;
    };

    class Function : public Block
    {
    public:
        virtual ~Function() {};
        virtual void Update() = 0;
    };

    class Sink : public Block
    {
    public:
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

        // virtual void Configure(inputs, output) = 0;
        virtual BlockList Blocks() = 0;
    };

    class Model {

    public:

        Model(float dtMax=0.1) : m_Configured(false), m_dtMax(dtMax) {}

        void Initialise(float t_0);
        void Update(float t_np1);


    protected:
        void RegisterBlocks(BlockList& blocks);

    private:
        void Configure();

        std::vector<Source*> m_Sources;
        std::vector<DynamicSystem*> m_DynamicSystems;
        std::vector<Function*> m_Functions;
        std::vector<Sink*> m_Sinks;

        bool m_Configured;
        float m_dtMax;
        float m_t_n;
    };







    //-------- Dynamic system and integration
    template <typename DerivativeType>
    class Integrable {
    public:
        virtual DerivativeType Derivative(float t, DerivativeType x) = 0;
    };

    class ForwardEuler {
    public:
        template<typename DerivativeType>
        static DerivativeType Step(Integrable<DerivativeType>* block, float dt, float t_n, DerivativeType x_n)
        {
            return x_n + dt * block->Derivative(t_n, x_n);
        };
    };

    class RK4 {
    public:
        template<typename DerivativeType>
        static DerivativeType Step(Integrable<DerivativeType>* block, float dt, float t_n, DerivativeType x_n)
        {
            DerivativeType k1 = dt * block->Derivative(t_n, x_n);
            DerivativeType k2 = dt * block->Derivative(t_n + dt / 2.f, x_n + k1 / 2.f);
            DerivativeType k3 = dt * block->Derivative(t_n + dt / 2.f, x_n + k2 / 2.f);
            DerivativeType k4 = dt * block->Derivative(t_n + dt, x_n + k3);

            return x_n + (1.f / 6.f) * (k1 + 2 * k2 + 2 * k3 + k4);
        };
    };


    namespace Internal {

        std::map<SignalBase*, std::vector<Function*>> FunctionInputs(std::vector<Function*> functions);
        std::vector<std::vector<int>> AdjacencyList(std::vector<Function*> functions);
        std::vector<int> TopologicalSort(std::vector<std::vector<int>> adjacencyList);
        std::vector<Function*> SortFunctions(std::vector<Function*> functions);

    }; // namespace Internal




} // namespace Framework

#endif //SIMFRAMEWORK_FRAMEWORK_H
