#ifndef SIMINTERFACE_MASS1D_H
#define SIMINTERFACE_MASS1D_H

#include "Eigen/Dense"
#include "Framework.h"

namespace Vehicle {

    class Inertia1D : public SimFramework::Block, public SimFramework::DynamicSystem<Eigen::Vector2f> {

    public:

        Inertia1D(SimFramework::Signal<float>* inputForcing, SimFramework::Signal<Eigen::Vector2f>* outputStates,
                  Eigen::Vector2f initialStates, float inertia = 1.f);

        // Block functions
        void Read() override;

        void Write() override;

        void Update(float t_np1) override;

        void Init(float t_0) override;

        // Dynamic system functions
        Eigen::Vector2f Gradient(float t, Eigen::Vector2f x) override;

    private:

        // Signals
        SimFramework::Signal<float>* m_InputForcing;
        SimFramework::Signal<Eigen::Vector2f>* m_OutputStates;

        // Internal copies
        Eigen::Vector2f m_InitStates;
        Eigen::Vector2f m_States;
        float m_InputCopy;
        float t_n = 0;

        // Physical Properties
        float m_Inertia;
    };

}; // namespace Vehicle


#endif //SIMINTERFACE_MASS1D_H
