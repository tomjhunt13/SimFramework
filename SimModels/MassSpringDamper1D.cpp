#include "SimModels/MassSpringDamper1D.h"


namespace Models {

    MassSpringDamper1D::MassSpringDamper1D(float mass, float k, float c) : Model(0.001), m_Mass(mass), m_K(k), m_C(c)
    {
        // Gain matrix
        Eigen::Matrix<float, 1, 2> gainMatrix;
        gainMatrix << this->m_K, this->m_C;

        // State space matrices
        Eigen::Matrix<float, 2, 2> A;
        A << 0.f, 1.f, 0.f, 0.f;

        Eigen::Vector2f B = {0.f, (1.f / this->m_Mass)};

        Eigen::Matrix<float, 2, 2> C;
        C << 1.f, 0.f, 0.f, 1.f;

        Eigen::Vector2f D = {0.f, 0.f};
        Eigen::Vector2f initialState = {0.05, 0.f};

        Eigen::Vector<float, 1> a;


        // Configure Blocks
        this->m_Input.Configure(&(this->m_InputForce), 0);
        this->m_Gain.Configure(&(this->m_MassStates), &(this->m_SpringDamperForce_Vec), gainMatrix);
        this->m_Mask1.Configure(&(this->m_SpringDamperForce_Vec), {&(this->m_SpringDamperForce)}, {0});
        this->m_SumForces.Configure({&(this->m_SpringDamperForce), &(this->m_InputForce)}, &(this->m_SummedForce), {-1.f, 1.f});
        this->m_MassBlock.Configure(&(this->m_SummedForce), &(this->m_MassStates), initialState);
        this->m_MassBlock.SetMatrices(A, B, C, D);
        this->m_Mask2.Configure(&(this->m_MassStates), {&(this->m_MassPosition), &(this->m_MassVelocity)}, {0, 1});
        this->m_PositionOutputBlock.Configure(&(this->m_MassPosition), initialState[0]);
        this->m_VelocityOutputBlock.Configure(&(this->m_MassVelocity), initialState[1]);


        // Construct system
        this->RegisterBlocks(
                {&(this->m_Input)},
                {&(this->m_MassBlock)},
                {&(this->m_Gain), &(this->m_Mask1), &(this->m_SumForces), &(this->m_Mask2)},
                {&(this->m_PositionOutputBlock), &(this->m_VelocityOutputBlock)});
    }

    SimFramework::Input<float>* MassSpringDamper1D::InputForceBlock()
    {
        return &(this->m_Input);
    };

    SimFramework::Output<float>* MassSpringDamper1D::MassPositionBlock()
    {
        return &(this->m_PositionOutputBlock);
    };

    SimFramework::Output<float>* MassSpringDamper1D::MassVelocityBlock()
    {
        return &(this->m_VelocityOutputBlock);
    };

}; // namespace SimModels