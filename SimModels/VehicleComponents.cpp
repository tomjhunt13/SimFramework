#include "SimModels/VehicleComponents.h"

namespace Models {

    void Clutch::Configure(SimFramework::Signal<float>* inEngineSpeed, SimFramework::Signal<float>* outClutchTorque)
    {
        this->m_InEngineSpeed = inEngineSpeed;
        this->m_OutClutchTorque = outClutchTorque;
    };

    std::vector<SimFramework::SignalBase*> Clutch::InputSignals()
    {
        return {this->m_InEngineSpeed};
    }

    std::vector<SimFramework::SignalBase*> Clutch::OutputSignals()
    {
        return {this->m_OutClutchTorque};
    }

    void Clutch::Update()
    {
        float speed = SimFramework::RadiansPerSecondToRPM(this->m_InEngineSpeed->Read()) / 1000.f;
        this->m_OutClutchTorque->Write(this->m_TorqueCapacity * ( speed * speed - this->m_EngagementSpeed * this->m_EngagementSpeed));
    };


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
                {},
                {});
    };





    LinearTrigger::LinearTrigger()
    {
        this->m_Default = 0.f;
        this->t_end = 1.f;
    }

    float LinearTrigger::Evaluate(float t) {
        return 1.f * (this->t_end - t) / (this->t_end);
    }

}; // namespace Models