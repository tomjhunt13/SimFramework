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

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update(float dt) override;

    private:
        ELockupClutchState m_LockState;

        const SimFramework::Signal<bool>* m_SpeedMatch;
        const SimFramework::Signal<float>* m_TransmittedTorque;
        const SimFramework::Signal<float>* m_ClutchTorqueLimit;
        LockupClutch* m_LockupModel;
    };



    class LockupClutch : SimFramework::Subsystem {
    public:

        // Set up functions
        void SetParameters(
                std::string engineJSON="/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json", float initialSpeed=200.f,
                std::vector<float> gearRatios={9, 6, 4.5, 3, 2, 1.5},
                float b_e=0.3, float b_t=0.3, float I_e=1.f, float I_t=1.f,
                float maxClutchTorque=250);

        void Configure(
                const SimFramework::Signal<float>* inThrottlePosition,
                const SimFramework::Signal<float>* inClutchPosition,
                const SimFramework::Signal<float>* inTyreTorque);


        // Transmission functions
        bool ShiftUp();
        bool ShiftDown();




        // Output signals
        const SimFramework::Signal<float>* OutEngineSpeed() const;
        const SimFramework::Signal<float>* OutWheelSpeed() const;
        const SimFramework::Signal<int>* OutGearIndex() const;
        const SimFramework::Signal<float>* OutFuelRate() const;
        const SimFramework::Signal<float>* OutFuelCumulative() const;

        // Subsystem functions
        SimFramework::BlockList LockupClutch::Blocks() override;
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override { return {}; };


        void TransitionState(ELockupClutchState newState);



    private:

        // Transmission
        void SetGearRatio(float G);
        std::vector<float> m_Ratios;
        int m_GearIndex;

        // State space parameters
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
        CoulombFriction m_MaxClutchTorque;

        // State space
        SimFramework::StateSpace<Eigen::Vector3f, Eigen::Vector2f, 3, 2, 2> m_UnLockedState;
        SimFramework::StateSpace<Eigen::Vector2f, Eigen::Vector2f, 2, 1, 2> m_LockedState;
        SimFramework::StateSpace<float, Eigen::Vector<float, 1>, 1, 1, 1> m_FuelIntegrator;

        // Vector inputs
        SimFramework::Vectorise<float, Eigen::Vector3f> m_UnlockedInput;
        SimFramework::Vectorise<float, Eigen::Vector2f> m_LockedInput;

        // Mask outputs
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_UnlockedMask;
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_LockedMask;
        SimFramework::Mask<Eigen::Vector<float, 1>, float, 1> m_FuelUsageMask;

        // Switches
        SimFramework::Switch<float> m_EngineSpeedSwitch;
        SimFramework::Switch<float> m_WheelSpeedSwitch;

        // Lock state manager
        CrossingDetect m_CrossingDetect;
        LockupClutchController m_LockStateController;

        // Gear index
        SimFramework::Input<int> m_InGearIndex;

    };

}; // namespace Models


#endif //FRAMEWORK_LOCKUPCLUTCH_H
