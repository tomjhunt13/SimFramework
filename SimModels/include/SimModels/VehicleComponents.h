#ifndef FRAMEWORK_VEHICLECOMPONENTS_H
#define FRAMEWORK_VEHICLECOMPONENTS_H


#include "SimFramework/Utilities.h"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

namespace Models {

    class Clutch : public SimFramework::Function
    {
    public:
        void Configure(SimFramework::Signal<float>* inEngineSpeed, SimFramework::Signal<float>* outClutchTorque);

        std::vector<SimFramework::SignalBase*> InputSignals() override;
        std::vector<SimFramework::SignalBase*> OutputSignals() override;
        void Update() override;

    private:
        // Parameters
        float m_EngagementSpeed = 1.5f; // [1000 rev/min]
        float m_TorqueCapacity = 30.f; // [Nm @ engagement speed]

        // Signals
        SimFramework::Signal<float>* m_InEngineSpeed;
        SimFramework::Signal<float>* m_OutClutchTorque;

        // Copies
        float m_InSpeedCopy;
        float m_OutTorqueCopy;
    };

    class Engine : public SimFramework::Subsystem {
    public:

        void SetEngineParameters(std::string engineJSON="/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json", float J=1.f, float b=0.05);

        void Configure(
                SimFramework::Signal<float>* inThrottle,
                SimFramework::Signal<float>* inLoadTorque,
                SimFramework::Signal<float>* outEngineSpeed);

        void RegisterBlocks(SimFramework::Model* model) override;

    private:
        // Signals
        SimFramework::Signal<Eigen::Matrix<float, 1, 1>> m_SEngineSpeed_;
        SimFramework::Signal<float> m_SEngineTorque;
        SimFramework::Signal<float> m_SResultantTorque;

        // Blocks
        SimFramework::LookupTable2D m_BEngineMap;
        SimFramework::SummingJunction<float> m_BSum;
        SimFramework::StateSpace<float, Eigen::Matrix<float, 1, 1>, 1, 1, 1> m_BInertia;
        SimFramework::Mask<Eigen::Matrix<float, 1, 1>, float> m_BMask;
    };




    class LinearTrigger : public SimFramework::TriggerFunction {
    public:
        LinearTrigger();
        float Evaluate(float t);
    };

}; // namespace Models


#endif //FRAMEWORK_VEHICLECOMPONENTS_H
