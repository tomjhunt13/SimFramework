#include "SimModels/Engine.h"

namespace Models {

    Engine::Engine() : m_EngineMap("Engine Map"), m_TorqueVector("Engine Torque"), m_Inertia("Engine"), m_SpeedMask("Engine") {};

    void Engine::SetParameters(std::string engineJSON, float initialSpeed, float J, float b)
    {
        // Set engine table
        SimFramework::Table3D engineTable = SimFramework::ReadTableJSON(engineJSON, "speed", "throttle", "torque");
        this->m_EngineMap.SetTable(engineTable);

        // Set state space matrices
        Eigen::Matrix<float, 1, 1> A;
        A << -b / J;

        Eigen::Matrix<float, 1, 2> B;
        B << 1.f / J, -1.f / J;

        Eigen::Matrix<float, 1, 1> C;
        C << 1.f;

        Eigen::Matrix<float, 1, 2> D;
        D << 0.f, 0.f;

        this->m_Inertia.SetMatrices(A, B, C, D);

        // Initial engine speed
        Eigen::Matrix<float, 1, 1> init;
        init << initialSpeed;
        this->m_Inertia.SetInitialConditions(init);
    }

    void Engine::Configure(
            const SimFramework::Signal<float>* inThrottle,
            const SimFramework::Signal<float>* inLoadTorque)
    {
        // Configure blocks
        this->m_EngineMap.Configure(this->OutEngineSpeed(), inThrottle);
        this->m_TorqueVector.Configure({this->m_EngineMap.OutSignal(), inLoadTorque});
        this->m_Inertia.Configure(this->m_TorqueVector.OutSignal());
        this->m_SpeedMask.Configure(this->m_Inertia.OutSignal());
    };

    const SimFramework::Signal<float>* Engine::OutEngineSpeed() const
    {
        return this->m_SpeedMask.OutSignal(0);
    };


    SimFramework::BlockList Engine::Blocks()
    {
        // Construct system
        return {{},
                {&(this->m_Inertia)},
                {&(this->m_SpeedMask), &(this->m_EngineMap), &(this->m_TorqueVector)},
                {},
                {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Engine::LogSignals()
    {
        return {{"Engine Speed", this->m_Inertia.OutSignal()},
                {"Engine Torque, Engine Clutch Torque", this->m_TorqueVector.OutSignal()}};
    };

}; // namespace Models