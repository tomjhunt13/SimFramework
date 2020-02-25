#ifndef SIMFRAMEWORK_FRAMEWORK_H
#define SIMFRAMEWORK_FRAMEWORK_H

#include <vector>
#include <map>
#include <string>

#include "Eigen/Dense"

namespace SimFramework {


    //----------------- Signal
    template <typename SignalType>
    class Signal
    {
    public:
        Signal(std::string name = "Signal") : m_Name(name) {};

        SignalType Read() const { return this->m_Value; };
        void Write(SignalType value) { this->m_Value = value; };

    private:
        SignalType m_Value;
        std::string m_Name;
    };


    //----------------- Block
    class Block
    {
    public:
        virtual ~Block() {};

        // Block API
        virtual void Read() = 0;
        virtual void Write() = 0;
        virtual void Update(float t_np1) = 0;
        virtual void Init(float t_0) = 0;
    };


    //-------- Dynamic system and integration
    template <typename GradientType>
    class DynamicSystem {
    public:
        virtual GradientType Gradient(float t, GradientType x) = 0;
    };

    class ForwardEuler {
    public:
        template<typename GradientType>
        static GradientType Step(DynamicSystem<GradientType> & block, float dt, float t_n, GradientType &x_n)
        {
            return x_n + dt * block.Gradient(t_n, x_n);
        };
    };

    //----------------- SystemManager


    class SystemManager {

    public:

        // Copy constructor deleted according to singleton design pattern
        SystemManager(SystemManager &systemManager) = delete;

        static SystemManager& Get();

        void RegisterBlocks(std::vector<Block*> sources, std::vector<Block*> dynamicSystems,
                                   std::vector<Block*> functions, std::vector<Block*> sinks);

        void Initialise(float t_0);

        // Solution phase
        void UpdateSystem(float t_np1);

    private:

        void UpdateFunctions(float t_np1);

        std::vector<Block*> m_Sources;
        std::vector<Block*> m_DynamicSystems;
        std::vector<Block*> m_Functions;
        std::vector<Block*> m_Sinks;

        // Constructor hidden to maintain singleton pattern
        SystemManager() = default;
    };




} // namespace Framework

#endif //SIMFRAMEWORK_FRAMEWORK_H
