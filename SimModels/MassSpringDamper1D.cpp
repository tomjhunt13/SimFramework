#include "SimModels/MassSpringDamper1D.h"


namespace Models {

    MassSpringDamper1D::MassSpringDamper1D(float mass, float k, float c) : System(0.001), m_Mass(mass), m_K(k), m_C(c)
    {

        // Configure blocks
        this->m_Input.Configure(0.f);
        this->m_StateSpace.Configure(this->m_Input.OutSignal());
        this->m_StateMask.Configure(this->m_StateSpace.OutSignal());
        this->m_PositionOutputBlock.Configure(this->m_StateMask.OutSignal(0), 0.f);
        this->m_VelocityOutputBlock.Configure(this->m_StateMask.OutSignal(1), 0.f);

        // Set system state space matrices
        Eigen::Matrix<float, 2, 2> A;
        A << 0.f, 1.f, -k / mass, - c / mass;

        Eigen::Vector2f B = {0.f, (1.f / this->m_Mass)};

        Eigen::Matrix<float, 2, 2> C;
        C << 1.f, 0.f, 0.f, 1.f;

        Eigen::Vector2f D = {0.f, 0.f};

        Eigen::Vector2f initialState = {0.05, 0.f};

        this->m_StateSpace.SetMatrices(A, B, C, D);
        this->m_StateSpace.SetInitialConditions(initialState);

        // Register blocks
        SimFramework::BlockList list = {{&(this->m_Input)},
                                        {&(this->m_StateSpace)},
                                        {&(this->m_StateMask)},
                                        {&(this->m_PositionOutputBlock), &(this->m_VelocityOutputBlock)},
                                        {}};

        this->RegisterBlocks(list);

        this->LogSignal("Position", this->m_StateMask.OutSignal(0));
        this->LogSignal("Velocity", this->m_StateMask.OutSignal(1));
        this->LogSignal("Input Force", this->m_Input.OutSignal());
        this->LogSignal("States(0),States(1)", this->m_StateSpace.OutSignal());
    }

    MassSpringDamperBlocks MassSpringDamper1D::Blocks()
    {
        return {&(this->m_Input), &(this->m_PositionOutputBlock), &(this->m_VelocityOutputBlock)};
    }

}; // namespace SimModels