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
        static GradientType Step(DynamicSystem<GradientType>* block, float dt, float t_n, GradientType x_n)
        {
            return x_n + dt * block->Gradient(t_n, x_n);
        };
    };

    class RK4 {
    public:
        template<typename GradientType>
        static GradientType Step(DynamicSystem<GradientType>* block, float dt, float t_n, GradientType x_n)
        {
            GradientType k1 = dt * block->Gradient(t_n, x_n);
            GradientType k2 = dt * block->Gradient(t_n + dt / 2.f, x_n + k1 / 2.f);
            GradientType k3 = dt * block->Gradient(t_n + dt / 2.f, x_n + k2 / 2.f);
            GradientType k4 = dt * block->Gradient(t_n + dt, x_n + k3);

            return x_n + (1.f / 6.f) * (k1 + 2 * k2 + 2 * k3 + k4);
        };
    };




    class Model {

    public:

        void Initialise(float t_0);
        void Update(float t_np1);

    protected:
        void RegisterBlocks(std::vector<Block*> sources, std::vector<Block*> dynamicSystems,
                                   std::vector<Block*> functions, std::vector<Block*> sinks);

    private:
        void UpdateFunctions(float t_np1);
        std::vector<Block*> m_Sources;
        std::vector<Block*> m_DynamicSystems;
        std::vector<Block*> m_Functions;
        std::vector<Block*> m_Sinks;
    };


} // namespace Framework

#endif //SIMFRAMEWORK_FRAMEWORK_H
