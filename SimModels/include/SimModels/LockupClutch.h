#ifndef FRAMEWORK_LOCKUPCLUTCH_H
#define FRAMEWORK_LOCKUPCLUTCH_H

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

    struct EngineTransmissionParameters {
        float G;
        float b_e;
        float b_t;
        float I_e;
        float I_t;
    };


    class TransmittedTorque : public SimFramework::Function
    {
    public:
        TransmittedTorque(std::string name="Transmitted Torque");

        void SetParameters(EngineTransmissionParameters parameters);
        void Configure(
                const SimFramework::Signal<float>* inSpeed,
                const SimFramework::Signal<float>* inEngineTorque,
                const SimFramework::Signal<float>* inTyreTorque);
        const SimFramework::Signal<float>* OutTorque() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:
        float Evaluate(float w, float T_e, float T_t, float G, float b_e, float b_t, float I_e, float I_t);

        // Parameters
        EngineTransmissionParameters m_Parameters;

        // Signals
        const SimFramework::Signal<float>* m_InSpeed;
        const SimFramework::Signal<float>* m_InEngineTorque;
        const SimFramework::Signal<float>* m_InTyreTorque;
        SimFramework::Signal<float> m_OutTorque;
    };


    class LockupClutchController : SimFramework::Sink {
    public:

        void Configure(
                const SimFramework::Signal<bool>* inSpeedMatch,
                const SimFramework::Signal<float>* inTransmittedTorque,
                const SimFramework::Signal<float>* inClutchTorqueLimit,
                LockupClutch* lockupModel);

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update(float dt) override;

    private:
        ELockupClutchState m_LockState;

        const SimFramework::Signal<bool>* m_SpeedMatch;
        const SimFramework::Signal<bool>* m_TransmittedTorque;
        const SimFramework::Signal<bool>* m_ClutchTorqueLimit;
        LockupClutch* m_LockupModel;
    };



    class LockupClutch : SimFramework::Subsystem {
    public:

        void Configure(
                const SimFramework::Signal<float>* inThrottlePosition,
                const SimFramework::Signal<float>* inClutchPosition,
                const SimFramework::Signal<float>* inTyreTorque);


        void TransitionState(ELockupClutchState newState) {};

        void SetGearRatio(float G);


    private:

        // System Parameters
        float b_e = 0.2;
        float b_t = 0.5;
        float I_e = 0.25;
        float I_t = 0.4;

        // Engine maps
        SimFramework::LookupTable2D m_TorqueMap;
        SimFramework::LookupTable2D m_FuelMap;

        // Clutch
        TransmittedTorque m_TransmittedTorque;
        SimFramework::SummingJunction<float> m_RelativeSpeed;
        SimFramework::Gain<float, float, float> m_ClutchGain;
        CoulombFriction m_MaxClutchTorque;

        // State space
        SimFramework::StateSpace<Eigen::Vector3f, Eigen::Vector2f, 3, 2, 2> m_UnLockedState;
        SimFramework::StateSpace<Eigen::Vector2f, Eigen::Vector2f, 2, 1, 2> m_LockedState;

        // Vector inputs
        SimFramework::Vectorise<float, Eigen::Vector3f> m_UnlockedInput;
        SimFramework::Vectorise<float, Eigen::Vector2f> m_LockedInput;

        // Mask outputs
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_UnlockedMask;
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_LockedMask;

        // Switches
        SimFramework::Switch<float> m_EngineSpeedSwitch;






    };

}; // namespace Models


#endif //FRAMEWORK_LOCKUPCLUTCH_H
