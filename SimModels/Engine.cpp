#include "SimModels/Engine.h"

namespace Models {

    Engine::Engine() : m_TorqueMap("Engine Torque Map"), m_FuelMap("Engine Fuel Map"), m_InputVector("Engine Input"), m_StateSpace("Engine"), m_OutputMask("Engine") {};

    void Engine::SetParameters(std::string engineJSON, float initialSpeed, float J, float b)
    {
        // Set engine table
        SimFramework::Table3D torqueTable = SimFramework::ReadTableJSON(engineJSON, "speed", "throttle", "torque");
        this->m_TorqueMap.SetTable(torqueTable);

        SimFramework::Table3D fuelTable = SimFramework::ReadTableJSON(engineJSON, "speed", "throttle", "fuel");
        this->m_FuelMap.SetTable(fuelTable);

        // Set state space matrices
        Eigen::Matrix<float, 2, 2> A;
        A << -b / J, 0.f, 0.f, 0.f;

        Eigen::Matrix<float, 2, 3> B;
        B << 1.f / J, -1.f / J, 0.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<float, 2, 2> C;
        C << 1.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<float, 2, 3> D;
        D << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;

        this->m_StateSpace.SetMatrices(A, B, C, D);

        // Initial engine speed
        Eigen::Vector2f init;
        init << initialSpeed, 0.f;
        this->m_StateSpace.SetInitialConditions(init);
    }

    void Engine::Configure(
            const SimFramework::Signal<float>* inThrottle,
            const SimFramework::Signal<float>* inLoadTorque)
    {
        // Configure blocks
        this->m_TorqueMap.Configure(this->OutEngineSpeed(), inThrottle);
        this->m_FuelMap.Configure(this->OutEngineSpeed(), inThrottle);
        this->m_InputVector.Configure({this->m_TorqueMap.OutSignal(), inLoadTorque, });
        this->m_StateSpace.Configure(this->m_InputVector.OutSignal());
        this->m_OutputMask.Configure(this->m_StateSpace.OutSignal());
    };

    const SimFramework::Signal<float>* Engine::OutEngineSpeed() const
    {
        return this->m_OutputMask.OutSignal(0);
    };

    const SimFramework::Signal<float>* Engine::OutFuelRate() const
    {
        return this->m_FuelMap.OutSignal();
    };

    const SimFramework::Signal<float>* Engine::OutFuelCumulative() const
    {
        return this->m_OutputMask.OutSignal(1);
    };


    SimFramework::BlockList Engine::Blocks()
    {
        // Construct system
        return {{},
                {&(this->m_StateSpace)},
                {&(this->m_InputVector), &(this->m_TorqueMap), &(this->m_FuelMap), &(this->m_OutputMask)},
                {},
                {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Engine::LogSignals()
    {
        return {{"Engine Speed", this->OutEngineSpeed()},
                {"Engine Torque, Engine Clutch Torque, Engine Fuel Rate", this->m_InputVector.OutSignal()},
                {"Engine Cumulative Fuel", this->OutFuelCumulative()}};
    };

}; // namespace Models