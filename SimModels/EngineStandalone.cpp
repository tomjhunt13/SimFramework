#include "SimModels/EngineStandalone.h"


namespace Models {

    void Engine::SetEngineParameters(std::string engineJSON, float J, float b)
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

        // Initial engine speed
        Eigen::Matrix<float, 1, 1> init;
        init << 300.f;
        this->m_BInertia.SetInitialConditions(init);
    }

    void Engine::Configure(
            SimFramework::Signal<float>* inThrottle,
            SimFramework::Signal<float>* inLoadTorque,
            SimFramework::Signal<float>* outEngineSpeed)
    {
        // Configure blocks
        this->m_BEngineMap.Configure(outEngineSpeed, inThrottle, &(this->m_SEngineTorque));
        this->m_BSum.Configure({&(this->m_SEngineTorque), inLoadTorque}, &(this->m_SResultantTorque), {1.f, -1.f});
        this->m_BInertia.Configure(&(this->m_SResultantTorque), &(this->m_SEngineSpeed_));
        this->m_BMask.Configure(&(this->m_SEngineSpeed_), {outEngineSpeed}, {0});
    };

    void Engine::RegisterBlocks(SimFramework::Model* model)
    {
        // Construct system
        model->RegisterBlocks(
                {},
                {&(this->m_BInertia)},
                {&(this->m_BMask), &(this->m_BEngineMap), &(this->m_BSum)},
                {});
    };



    EngineStandalone::EngineStandalone() : Model(0.001)
    {
        // Configure blocks
        this->m_BThrottle.Configure(&(this->m_SThrottle), 0.f);
        this->m_BLoad.Configure(&(this->m_SLoadTorque), 0.f);
        this->m_BEngineSpeed.Configure(&(this->m_SEngineSpeed), 0.f);

        // Configure engine
        this->m_SysEngine.Configure(&(this->m_SThrottle), &(this->m_SLoadTorque), &(this->m_SEngineSpeed));
        this->m_SysEngine.RegisterBlocks(this);

        // Construct system
        this->RegisterBlocks(
                {&(this->m_BThrottle), &(this->m_BLoad)},
                {},
                {},
                {&(this->m_BEngineSpeed)});

        this->m_tempConst.Configure(&(this->m_STempConst), 40.f);
        this->m_tempSum.Configure({&(this->m_SEngineSpeed), &(this->m_STempConst)}, &(this->m_STempSum), {1.f, 1.f});
        this->m_tempGain.Configure(&(this->m_STempSum), &(this->m_STempGain), 3.f);
        this->m_tempOut.Configure(&(this->m_STempGain), 0.f);

        this->RegisterBlocks(
                {&(this->m_tempConst)},
                {},
                {&(this->m_tempGain), &(this->m_tempSum)},
                {&(this->m_tempOut)});

    }

    void EngineStandalone::SetEngineParameters(std::string engineJSON, float J, float b)
    {
        this->m_SysEngine.SetEngineParameters(engineJSON, J, b);
    }

    EngineBlocks EngineStandalone::Blocks()
    {
        return {&(this->m_BLoad), &(this->m_BThrottle), &(this->m_tempOut)};
    };


}; // namespace Models