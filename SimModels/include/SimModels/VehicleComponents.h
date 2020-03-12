#ifndef FRAMEWORK_VEHICLECOMPONENTS_H
#define FRAMEWORK_VEHICLECOMPONENTS_H


#include "SimFramework/Utilities.h"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

namespace Models {


//    class Clutch : public SimFramework::Block
//    {
//    public:
//        void Configure(SimFramework::Signal<float>* inEngineSpeed, SimFramework::Signal<float>* outClutchTorque);
//
//        // Block API
//        void Read() override;
//        void Write() override;
//        void Update(float dt) override;
//        void Init(float t_0) override;
//
//    private:
//        // Parameters
//        float m_EngagementSpeed = 1.5f; // [1000 rev/min]
//        float m_TorqueCapacity = 30.f; // [Nm @ engagement speed]
//
//        // Signals
//        SimFramework::Signal<float>* m_InEngineSpeed;
//        SimFramework::Signal<float>* m_OutClutchTorque;
//
//        // Copies
//        float m_InSpeedCopy;
//        float m_OutTorqueCopy;
//    };
//
    class LinearTrigger : public SimFramework::TriggerFunction {
    public:
        LinearTrigger();
        float Evaluate(float t);
    };

}; // namespace Models


#endif //FRAMEWORK_VEHICLECOMPONENTS_H
