#include "SimModels/EngineStandalone.h"


namespace Models {

    EngineStandalone::EngineStandalone() : Model(0.001)
    {

        // TODO: Put this bit elsewhere
        // Initial engine speed
        Eigen::Matrix<float, 1, 1> init;
        init << 300.f;

        // Configure blocks
        this->m_BThrottle.Configure(&(this->m_SThrottle), 0.f);
        this->m_BLoad.Configure(&(this->m_SLoadTorque), 0.f);
        this->m_BEngineSpeed.Configure(&(this->m_SEngineSpeed), 0.f);
        this->m_BEngineMap.Configure(&(this->m_SEngineSpeed), &(this->m_SThrottle), &(this->m_SEngineTorque));
        this->m_BSum.Configure({&(this->m_SEngineTorque), &(this->m_SLoadTorque)}, &(this->m_SResultantTorque), {1.f, -1.f});
        this->m_BInertia.Configure(&(this->m_SResultantTorque), &(this->m_SEngineSpeed_), init);
        this->m_BMask.Configure(&(this->m_SEngineSpeed_), {&(this->m_SEngineSpeed)}, {0});

        // Construct system
        this->RegisterBlocks(
                {&(this->m_BThrottle), &(this->m_BLoad)},
                {&(this->m_BInertia)},
                {&(this->m_BMask), &(this->m_BEngineMap), &(this->m_BSum)},
                {&(this->m_BEngineSpeed)});

    }

    void EngineStandalone::SetEngineParameters(std::string engineJSON, float J, float b)
    {
        // Set engine table
        SimFramework::Table3D engineTable = SimFramework::ReadTableJSON(engineJSON, "speed", "throttle", "torque");
        this->m_BEngineMap.SetTable(engineTable);

        // Set state space matrices
        Eigen::Matrix<float, 1, 1> A;
        A << -b;

        Eigen::Matrix<float, 1, 1> B;
        B << 1.f / J;

        Eigen::Matrix<float, 1, 1> C;
        C << 1.f;

        Eigen::Matrix<float, 1, 1> D;
        D << 1.f;

        this->m_BInertia.SetMatrices(A, B, C, D);
    }

    EngineBlocks EngineStandalone::Blocks()
    {
        return {&(this->m_BLoad), &(this->m_BThrottle), &(this->m_BEngineSpeed)};
    };

    SimFramework::Input<float>* EngineStandalone::InputLoadBlock()
    {
        return &(this->m_BLoad);
    };

    SimFramework::Input<float>* EngineStandalone::InputThrottleBlock()
    {
        return &(this->m_BThrottle);
    };

    SimFramework::Output<float>* EngineStandalone::OutputSpeedBlock()
    {
        return &(this->m_BEngineSpeed);
    };

}; // namespace Models