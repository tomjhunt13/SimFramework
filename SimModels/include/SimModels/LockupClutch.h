#ifndef FRAMEWORK_LOCKUPCLUTCH_H
#define FRAMEWORK_LOCKUPCLUTCH_H

#include <cmath>

#include "Eigen/Dense"

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

#include "SimModels/VehicleComponents.h"

namespace Models {

    class LockupClutch;


    enum class ELockupClutchState {
        e_Locked,
        e_Unlocked
    };


    class TransmittedTorque : public SimFramework::Function
    {
    public:
        TransmittedTorque(std::string name="Transmitted Torque");

        void SetParameters(float b_1, float b_2, float I_1, float I_2);
        void Configure(
                const SimFramework::Signal<float>* inSpeed,
                const SimFramework::Signal<float>* inTorque1,
                const SimFramework::Signal<float>* inTorque2);
        const SimFramework::Signal<float>* OutTorque() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:

        // Parameters
        float b_1;
        float b_2;
        float I_1;
        float I_2;

        // Signals
        const SimFramework::Signal<float>* m_InSpeed;
        const SimFramework::Signal<float>* m_InTorque1;
        const SimFramework::Signal<float>* m_InTorque2;
        SimFramework::Signal<float> m_OutTorque;
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

        void Configure(
                const SimFramework::Signal<bool>* inSpeedMatch,
                const SimFramework::Signal<float>* inTransmittedTorque,
                const SimFramework::Signal<float>* inClutchTorqueLimit,
                LockupClutch* lockupModel);

        const SimFramework::Signal<int>* OutState() const;
        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update(float dt) override;

    private:
        ELockupClutchState m_LockState;

        const SimFramework::Signal<bool>* m_SpeedMatch;
        const SimFramework::Signal<float>* m_TransmittedTorque;
        const SimFramework::Signal<float>* m_ClutchTorqueLimit;
        SimFramework::Signal<int> m_StateSignal;
        LockupClutch* m_LockupModel;
    };



    class LockupClutch : public SimFramework::Subsystem {
    public:

        void SetParameters(float initSpeed1=0.f, float initSpeed2=0.f, float b_1=1.f, float b_2=1.f, float I_1=1.f, float I_2=1.f, float peakClutchTorque = 400.f);

        void Configure(
                const SimFramework::Signal<float>* inTorque1,
                const SimFramework::Signal<float>* inTorque2,
                const SimFramework::Signal<float>* inClutchEngagement);


        // Output signals
        const SimFramework::Signal<float>* OutSpeed1() const;
        const SimFramework::Signal<float>* OutSpeed2() const;

        // Subsystem functions
        SimFramework::BlockList Blocks() override;
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;


        void TransitionState(ELockupClutchState newState);


    private:

        void UpdateSSMatrices(float b_1=1.f, float b_2=2, float I_1=1.f, float I_2=1.f);


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
        TransmittedTorque m_TransmittedTorque;
        CrossingDetect m_CrossingDetect;
        LockupClutchController m_LockStateController;

        // Clutch
        CoulombFriction m_ClutchTorque;
    };

}; // namespace Models


#endif //FRAMEWORK_LOCKUPCLUTCH_H
