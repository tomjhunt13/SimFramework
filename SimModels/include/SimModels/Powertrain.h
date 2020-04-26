#ifndef FRAMEWORK_POWERTRAIN_H
#define FRAMEWORK_POWERTRAIN_H

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

#include "SimModels/VehicleComponents.h"

namespace Models {

    class Powertrain;

    enum class ELockupClutchState {
        e_Locked,
        e_Unlocked
    };


    class CrossingDetect : public SimFramework::Function
    {
    public:
        CrossingDetect(std::string name="Crossing Detect");

        void SetParameters(float offset, bool initialSign);
        void Configure(const SimFramework::Signal<float>* inSignal);
        const SimFramework::Signal<bool>* OutCrossing() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:
        // Parameters
        float m_Offset;
        bool m_PreviousPositive;

        // Signals
        const SimFramework::Signal<float>* m_InSignal;
        SimFramework::Signal<bool> m_Crossing;
    };


    class LockupClutchController : public SimFramework::Sink {
    public:

        LockupClutchController(std::string name="Lockup Clutch Controller");

        void SetParameters(float G, float b_e, float b_w, float I_e, float I_w);

        void Configure(
                const SimFramework::Signal<float>* inSpeed,
                const SimFramework::Signal<float>* inTorqueEngine,
                const SimFramework::Signal<float>* inTorqueWheel,
                const SimFramework::Signal<bool>* inSpeedMatch,
                const SimFramework::Signal<float>* inClutchTorqueLimit,
                Powertrain* lockupModel);

        const SimFramework::Signal<int>* OutState() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update(float dt) override;

    private:

        // Parameters - Transmitted Torque
        float G;
        float b_e;
        float b_w;
        float I_e;
        float I_w;

        // Parameters - State
        ELockupClutchState m_LockState;
        Powertrain* m_LockupModel;

        // Signals - Transmitted torque
        const SimFramework::Signal<float>* m_ShaftSpeed;
        const SimFramework::Signal<float>* m_TorqueEngine;
        const SimFramework::Signal<float>* m_TorqueWheel;

        // Signals - State
        const SimFramework::Signal<bool>* m_SpeedMatch;
        const SimFramework::Signal<float>* m_ClutchTorqueLimit;
        SimFramework::Signal<int> m_StateSignal;
    };

    class Powertrain : public SimFramework::Subsystem {
    public:

        void SetParameters(float initSpeed1=0.f, float initSpeed2=0.f, float b_1=1.f, float b_2=1.f, float I_1=1.f, float I_2=1.f, float MaxNormalForce=100.f, float ClutchTorqueCapacity = 400.f);

        void Configure(
                const SimFramework::Signal<float>* inTorqueEngine,
                const SimFramework::Signal<float>* inTorqueWheel,
                const SimFramework::Signal<float>* inClutchEngagement);


        // Output signals
        const SimFramework::Signal<float>* OutSpeed1() const;
        const SimFramework::Signal<float>* OutSpeed2() const;
        const SimFramework::Signal<int>* OutEngagement() const;

        // Subsystem functions
        SimFramework::BlockList Blocks() override;
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;


        void TransitionState(ELockupClutchState newState);


    private:

        void UpdateSSMatrices(float G, float b_e, float b_w, float I_e, float I_w);


        // Vector inputs
        SimFramework::Vectorise<float, Eigen::Vector3f> m_UnlockedInput;
        SimFramework::Vectorise<float, Eigen::Vector2f> m_LockedInput;

        // State space
        SimFramework::StateSpace<Eigen::Vector3f, Eigen::Vector2f, 3, 2, 2> m_UnLockedState;
        SimFramework::StateSpace<Eigen::Vector2f, Eigen::Vector2f, 2, 1, 2> m_LockedState;

        // Switch
        SimFramework::Switch<Eigen::Vector2f> m_Switch;

        // Mask outputs
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_StateMask;
        SimFramework::SummingJunction<float> m_RelativeSpeed;

        // Lock state manager
        CrossingDetect m_CrossingDetect;
        LockupClutchController m_LockStateController;

        // Clutch
        SimFramework::Gain<float, float, float> m_ClutchTorqueCapacity;
        CoulombFriction m_SignedClutchTorque;
    };

}; // namespace Models


#endif //FRAMEWORK_POWERTRAIN_H
