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
        // Block API
        virtual void Read() = 0;
        virtual void Write() = 0;
        virtual void Update(float t_np1) = 0;
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

        static void RegisterBlocks(std::vector<Block*> blocks);

        // Solution phase
        static void UpdateSystem(float t_np1);

    private:

        std::vector<Block*> m_Blocks;

        // Constructor hidden to maintain singleton pattern
        SystemManager() = default;
    };




} // namespace SimFramework

#endif //SIMFRAMEWORK_FRAMEWORK_H
