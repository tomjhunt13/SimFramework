#ifndef FRAMEWORK_MASSSPRINGDAMPER1D_H
#define FRAMEWORK_MASSSPRINGDAMPER1D_H

#include "Eigen/Dense"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"


namespace Models {

    struct MassSpringDamperBlocks
    {
        SimFramework::Input<float>* InputForceBlock;
        SimFramework::Output<float>* MassPositionBlock;
        SimFramework::Output<float>* MassVelocityBlock;
    };

    class MassSpringDamper1D : public SimFramework::System
    {

    public:
        MassSpringDamper1D(float mass=1.f, float k=1.f, float c=1.f);

        MassSpringDamperBlocks Blocks();

        SimFramework::Input<float>* InputForceBlock();
        SimFramework::Output<float>* MassPositionBlock();
        SimFramework::Output<float>* MassVelocityBlock();

    private:

        // Parameters
        float m_Mass;
        float m_K;
        float m_C;

        // Signals
        SimFramework::Signal<float> m_InputForce;
        SimFramework::Signal<Eigen::Vector<float, 1>> m_SpringDamperForce_Vec;
        SimFramework::Signal<float> m_SpringDamperForce;
        SimFramework::Signal<float> m_SummedForce;
        SimFramework::Signal<Eigen::Vector2f> m_MassStates;
        SimFramework::Signal<float> m_MassPosition;
        SimFramework::Signal<float> m_MassVelocity;

        // Blocks
        SimFramework::Input<float> m_Input;
        SimFramework::Gain<Eigen::Vector2f, Eigen::Vector<float, 1>, Eigen::Matrix<float, 1, 2>> m_Gain;
        SimFramework::Mask<Eigen::Vector<float, 1>, float, 1> m_Mask1;
        SimFramework::SummingJunction<float> m_SumForces;
        SimFramework::StateSpace<float, Eigen::Vector2f, 1, 2, 2> m_MassBlock;
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_Mask2;
        SimFramework::Output<float> m_PositionOutputBlock;
        SimFramework::Output<float> m_VelocityOutputBlock;
    };

}; // namespace SimModels


#endif //FRAMEWORK_MASSSPRINGDAMPER1D_H
