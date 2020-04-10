#include "SimModels/LockupClutch.h"

namespace Models {

    TransmittedTorque::TransmittedTorque(std::string name) : Function(name) {};

    void TransmittedTorque::SetParameters(EngineTransmissionParameters parameters)
    {
        this->m_Parameters = parameters;
    };

    void TransmittedTorque::Configure(
            const SimFramework::Signal<float>* inSpeed,
            const SimFramework::Signal<float>* inEngineTorque,
            const SimFramework::Signal<float>* inTyreTorque)
    {
        this->m_InSpeed = inSpeed;
        this->m_InEngineTorque = inEngineTorque;
        this->m_InTyreTorque = inTyreTorque;
    };

    const SimFramework::Signal<float>* TransmittedTorque::OutTorque() const
    {
        return &(this->m_OutTorque);
    };

    std::vector<const SimFramework::SignalBase*> TransmittedTorque::InputSignals() const
    {
        return {this->m_InTyreTorque, this->m_InEngineTorque, this->m_InSpeed};
    };

    std::vector<const SimFramework::SignalBase*> TransmittedTorque::OutputSignals() const
    {
        return {&(this->m_OutTorque)};
    };

    void TransmittedTorque::Update()
    {
        // Read inputs
        float speed = this->m_InSpeed->Read();
        float engineTorque = this->m_InEngineTorque->Read();
        float tyreTorque = this->m_InTyreTorque->Read();

        // Calculate transmitted torque
        float outTorque = this->Evaluate(speed, engineTorque, tyreTorque, this->m_Parameters.G, this->m_Parameters.b_e, this->m_Parameters.b_t, this->m_Parameters.I_e, this->m_Parameters.I_t);

        // Write to output signal
        this->m_OutTorque.Write(outTorque);
    };

    float TransmittedTorque::Evaluate(float w, float T_e, float T_t, float G, float b_e, float b_t, float I_e, float I_t)
    {
        return ((I_e * b_t - I_t * b_e) * w + I_t * T_e + I_e * T_t) / (I_t + G * I_e);
    };


    CrossingDetect::CrossingDetect(std::string name) : Function(name) {};

    void CrossingDetect::SetParameters(float offset)
    {
        this->m_Offset = offset;
        this->m_PreviousPositive = true;
    };

    void CrossingDetect::Configure(const SimFramework::Signal<float>* inSignal)
    {
        this->m_InSignal = inSignal;
    };

    const SimFramework::Signal<bool>* CrossingDetect::OutCrossing() const
    {
        return &(this->m_Crossing);
    };

    std::vector<const SimFramework::SignalBase*> CrossingDetect::InputSignals() const
    {
        return {this->m_InSignal};
    };

    std::vector<const SimFramework::SignalBase*> CrossingDetect::OutputSignals() const
    {
        return {this->OutCrossing()};
    };

    void CrossingDetect::Update()
    {
        // Read signal
        float newValue = this->m_InSignal->Read();

        // If signal greater than offset new signal value is positively signed
        bool newPositive = (newValue >= this->m_Offset);

        // If the sign has changed output is true
        bool output = (!newPositive == this->m_PreviousPositive);

        // Write output and transfer new value to old
        this->m_Crossing.Write(output);
        this->m_PreviousPositive = newPositive;
    };


    void LockupClutchController::Configure(
            const SimFramework::Signal<bool> *inSpeedMatch,
            const SimFramework::Signal<float> *inTransmittedTorque,
            const SimFramework::Signal<float>* inClutchTorqueLimit,
            LockupClutch* lockupModel)
    {
        this->m_SpeedMatch = inSpeedMatch;
        this->m_TransmittedTorque = inTransmittedTorque;
        this->m_ClutchTorqueLimit = inClutchTorqueLimit;
        this->m_LockupModel = lockupModel;
    }

    std::vector<const SimFramework::SignalBase*> LockupClutchController::InputSignals() const
    {
        return {this->m_SpeedMatch, this->m_TransmittedTorque, this->m_ClutchTorqueLimit};
    };

    std::vector<const SimFramework::SignalBase*> LockupClutchController::OutputSignals() const
    {
        return {};
    };

    void LockupClutchController::Update(float dt)
    {
        // Read inputs
        bool speedMatch = this->m_SpeedMatch->Read();
        float transmittedTorque = this->m_TransmittedTorque->Read();
        float torqueLimit = this->m_ClutchTorqueLimit->Read();

        switch (this->m_LockState)
        {
            case ELockupClutchState::e_Locked:
            {
                if (transmittedTorque > torqueLimit)
                {
                    this->m_LockupModel->TransitionState(ELockupClutchState::e_Unlocked);
                    this->m_LockState = ELockupClutchState::e_Unlocked;
                }

                break;
            }

            case ELockupClutchState::e_Unlocked:
            {
                if (transmittedTorque < torqueLimit && speedMatch)
                {
                    this->m_LockupModel->TransitionState(ELockupClutchState::e_Locked);
                    this->m_LockState = ELockupClutchState::e_Locked;
                }

                break;
            }
        }
    };



    void LockupClutch::Configure(
            const SimFramework::Signal<float>* inThrottlePosition,
            const SimFramework::Signal<float>* inClutchPosition,
            const SimFramework::Signal<float>* inTyreTorque)
    {
        // Engine Maps
        this->m_TorqueMap.Configure(this->m_EngineSpeedSwitch.OutSignal(), inThrottlePosition);
        this->m_FuelMap.Configure(this->m_EngineSpeedSwitch.OutSignal(), inThrottlePosition);

        // Clutch
        this->m_TransmittedTorque.Configure(this->m_EngineSpeedSwitch.OutSignal(), this->m_TorqueMap.OutSignal(), inTyreTorque);
        this->m_RelativeSpeed.Configure({this->m_UnlockedMask.OutSignal(1), this->m_UnlockedMask.OutSignal(0)}, {1.f, 0.f});
        this->m_ClutchGain.Configure(inClutchPosition, 100.f);
        this->m_MaxClutchTorque.Configure(this->m_RelativeSpeed.OutSignal(), this->m_ClutchGain.OutSignal());

        // State space
        this->m_UnLockedState.Configure(this->m_UnlockedInput.OutSignal());
        this->m_LockedState.Configure(this->m_LockedInput.OutSignal());

        // Vector inputs
        this->m_UnlockedInput.Configure({this->m_TorqueMap.OutSignal(), this->m_MaxClutchTorque.OutForce(), inTyreTorque});
        this->m_LockedInput.Configure({this->m_TorqueMap.OutSignal(), inTyreTorque});

        // Mask outputs
        this->m_UnlockedMask.Configure(this->m_UnLockedState.OutSignal());
        this->m_LockedState.Configure(this->m_LockedState.OutSignal());

        // Switched
        this->m_EngineSpeedSwitch.Configure({this->m_UnlockedMask.OutSignal(1), this->m_LockedMask.OutSignal(1)}, 0);

    }


    void LockupClutch::TransitionState(ELockupClutchState newState)
    {

    }

    void LockupClutch::SetGearRatio(float G)
    {

        // Set up state space matrices for locked state
        float inertia = G / (G * this->I_e + this->I_t);

        Eigen::Matrix<float, 1, 1> lockedA;
        lockedA << - (this->b_e + this->b_t) * inertia;

        Eigen::Matrix<float, 1, 2> lockedB;
        lockedB << inertia, - inertia;

        Eigen::Matrix<float, 2, 1> lockedC;
        lockedC << 1.f, G;

        Eigen::Matrix<float, 2, 2> lockedD = Eigen::Matrix<float, 2, 2>::Zero();

        this->m_LockedState.SetMatrices(lockedA, lockedB, lockedC, lockedD);

        // Set up state space matrices for unlocked state
        Eigen::Matrix<float, 2, 2> unlockedA;
        unlockedA << - this->b_e / this->I_e, 0.f, 0.f, - this->b_t / this->I_t;

        Eigen::Matrix<float, 2, 3> unlockedB;
        unlockedB << 1.f / this->I_e, - 1.f / this->I_e, 0.f, 0.f, G / this->I_t, - 1 / this->I_t;

        Eigen::Matrix<float, 2, 2> unlockedC;
        unlockedC << 1.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<float, 2, 3> unlockedD = Eigen::Matrix<float, 2, 3>::Zero();

        this->m_UnLockedState.SetMatrices(unlockedA, unlockedB, unlockedC, unlockedD);

    };

}; // namespace Models