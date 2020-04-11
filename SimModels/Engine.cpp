#include "SimModels/Engine.h"

namespace Models {

    Engine::Engine() {};

    void Engine::SetParameters(std::string engineJSON)
    {
        // Set engine table
        SimFramework::Table3D torqueTable = SimFramework::ReadTableJSON(engineJSON, "speed", "throttle", "torque");
        this->m_TorqueMap.SetTable(torqueTable);

        SimFramework::Table3D fuelTable = SimFramework::ReadTableJSON(engineJSON, "speed", "throttle", "fuel");
        this->m_FuelMap.SetTable(fuelTable);

        // Set up integrator
        Eigen::Matrix<float, 1, 1> A = Eigen::Matrix<float, 1, 1>::Zero();
        Eigen::Matrix<float, 1, 1> B;
        B << 1.f;
        Eigen::Matrix<float, 1, 1> C;
        C << 1.f;
        Eigen::Matrix<float, 1, 1> D = Eigen::Matrix<float, 1, 1>::Zero();
        this->m_FuelIntegrator.SetMatrices(A, B, C, D);
        Eigen::Vector<float, 1> init = Eigen::Vector<float, 1>::Zero();
        this->m_FuelIntegrator.SetInitialConditions(init);
    }

    void Engine::Configure(
            const SimFramework::Signal<float>* inThrottle,
            const SimFramework::Signal<float>* inSpeed)
    {
        // Configure blocks
        this->m_TorqueMap.Configure(inSpeed, inThrottle);
        this->m_FuelMap.Configure(inSpeed, inThrottle);
        this->m_FuelIntegratorInput.Configure({this->m_FuelMap.OutSignal()});
        this->m_FuelIntegrator.Configure(this->m_FuelIntegratorInput.OutSignal());
        this->m_FuelOutputMask.Configure(this->m_FuelIntegrator.OutSignal());
    };

    const SimFramework::Signal<float>* Engine::OutEngineTorque() const
    {
        return this->m_TorqueMap.OutSignal();
    };

    const SimFramework::Signal<float>* Engine::OutFuelRate() const
    {
        return this->m_FuelMap.OutSignal();
    };

    const SimFramework::Signal<float>* Engine::OutFuelCumulative() const
    {
        return this->m_FuelOutputMask.OutSignal(0);
    };


    SimFramework::BlockList Engine::Blocks()
    {
        // Construct system
        return {{},
                {&(this->m_FuelIntegrator)},
                {&(this->m_TorqueMap), &(this->m_FuelMap), &(this->m_FuelIntegratorInput), &(this->m_FuelOutputMask)},
                {},
                {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Engine::LogSignals()
    {
        return {};
    };

}; // namespace Models