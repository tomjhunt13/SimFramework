#ifndef FRAMEWORK_CLUTCH_H
#define FRAMEWORK_CLUTCH_H

#include "SimFramework/Framework.h"

namespace Vehicle {


    // TODO: Clutch could be an externally defined function block
    class Clutch : public SimFramework::Block {
    public:

        // Speeds in rad/s
        Clutch(SimFramework::Signal<float> *innerSpeed, SimFramework::Signal<float> *outerSpeed,
               SimFramework::Signal<float> *transmittedTorque);

        // Block API
        void Read() override;

        void Write() override;

        void Update(float t_np1) override;

        void Init(float t_0) override;

    private:
        // Clutch physical parameters
        float m_EngagementSpeed = 1; // x10^3 rev/min
        float m_TorqueCapacity = 100; // @ 1000 rpm

        // Signal
        SimFramework::Signal<float>* m_InnerSpeed;
        SimFramework::Signal<float>* m_OuterSpeed;
        SimFramework::Signal<float>* m_OutputTorque;

        // Signal copies
        float m_InnerSpeedCopy;
        float m_OuterSpeedCopy;
        float m_OutputTorqueCopy;
    };

}; // namespace Models


#endif //FRAMEWORK_CLUTCH_H
