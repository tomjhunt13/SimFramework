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

    private:

        // Parameters
        float m_Mass;
        float m_K;
        float m_C;

        // Blocks
        SimFramework::Input<float> m_Input;
        SimFramework::StateSpace<float, Eigen::Vector2f, 1, 2, 2> m_StateSpace;
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_StateMask;
        SimFramework::Output<float> m_PositionOutputBlock;
        SimFramework::Output<float> m_VelocityOutputBlock;
    };

}; // namespace SimModels


#endif //FRAMEWORK_MASSSPRINGDAMPER1D_H
