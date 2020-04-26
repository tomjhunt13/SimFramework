#include "SimModels/Powertrain.h"


namespace Models {


    float TransmittedTorque(float T_e, float T_w, float w, float G, float b_e, float b_w, float I_e, float I_w)
    {
        return (T_e * I_w + I_e * G * T_w + w * (I_e * b_w - I_w * b_e))/(I_e * G * G + I_w);
    }



    CrossingDetect::CrossingDetect(std::string name) : SimFramework::Function(name) {};

    void CrossingDetect::SetParameters(float offset, bool initialSign)
    {
        this->m_Offset = offset;
        this->m_PreviousPositive = initialSign;
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


    LockupClutchController::LockupClutchController(std::string name) : SimFramework::Sink(name), m_LockState(ELockupClutchState::e_Unlocked) {};

    void LockupClutchController::SetParameters(float G, float b_e, float b_w, float I_e, float I_w)
    {
        this->G = G;
        this->b_e = b_e;
        this->b_w= b_w;
        this->I_e = I_e;
        this->I_w = I_w;
    };

    void LockupClutchController::Configure(
            const SimFramework::Signal<float>* inSpeed,
            const SimFramework::Signal<float>* inTorqueEngine,
            const SimFramework::Signal<float>* inTorqueWheel,
            const SimFramework::Signal<bool>* inSpeedMatch,
            const SimFramework::Signal<float>* inClutchTorqueLimit,
            Powertrain* lockupModel)
    {
        this->m_ShaftSpeed = inSpeed;
        this->m_TorqueEngine = inTorqueEngine;
        this->m_TorqueWheel = inTorqueWheel;
        this->m_SpeedMatch = inSpeedMatch;
        this->m_ClutchTorqueLimit = inClutchTorqueLimit;
        this->m_LockupModel = lockupModel;
    }

    std::vector<const SimFramework::SignalBase*> LockupClutchController::InputSignals() const
    {
        return {this->m_SpeedMatch, this->m_ClutchTorqueLimit, this->m_ShaftSpeed, this->m_TorqueEngine, this->m_TorqueWheel};
    };

    std::vector<const SimFramework::SignalBase*> LockupClutchController::OutputSignals() const
    {
        return {};
    };

    void LockupClutchController::Update(float dt)
    {
        // Read inputs
        float shaftSpeed = this->m_ShaftSpeed->Read();
        float torqueEngine = this->m_TorqueEngine->Read();
        float torqueWheel = this->m_TorqueWheel->Read();
        bool speedMatch = this->m_SpeedMatch->Read();
        float torqueLimit = std::abs(this->m_ClutchTorqueLimit->Read());

        // Calculate transmitted torque
        float transmittedTorque = std::abs(TransmittedTorque(torqueEngine, torqueWheel, shaftSpeed, this->G, this->b_e, this->b_w, this->I_e, this->I_w));


        switch (this->m_LockState)
        {
            case ELockupClutchState::e_Locked:
            {
                if (transmittedTorque > torqueLimit)
                {
                    this->m_LockupModel->TransitionState(ELockupClutchState::e_Unlocked);
                    this->m_LockState = ELockupClutchState::e_Unlocked;
                }

                this->m_StateSignal.Write(1);

                break;
            }

            case ELockupClutchState::e_Unlocked:
            {
                if (transmittedTorque < torqueLimit && speedMatch)
                {
                    this->m_LockupModel->TransitionState(ELockupClutchState::e_Locked);
                    this->m_LockState = ELockupClutchState::e_Locked;
                }

                this->m_StateSignal.Write(0);

                break;
            }
        }
    };
    const SimFramework::Signal<int>* LockupClutchController::OutState() const
    {
        return &(this->m_StateSignal);
    };







    void Powertrain::SetParameters(std::vector<float> gearRatios, float initEngineSpeed, float initWheelSpeed, float b_e, float b_w, float I_e, float I_w, float ClutchTorqueCapacity)
    {
        // Set up gear ratios
        std::vector<float> ratios(1 + gearRatios.size());
        ratios[0] = gearRatios[0];
        for (int i = 0; i < gearRatios.size(); i++)
        {
            ratios[i+1] = gearRatios[i];
        }
        this->m_Ratios = ratios;

        this->m_GearIndex = 0;
        this->SetGearRatio();
        this->m_InGearIndex.WriteValue(this->m_GearIndex);

        // SS parameters
        this->b_e = b_e;
        this->b_w= b_w;
        this->I_e = I_e;
        this->I_w = I_w;

        this->m_CrossingDetect.SetParameters(0, true);

        // Set up clutch
        this->m_ClutchTorqueCapacity.SetGain(ClutchTorqueCapacity);
        this->m_SignedClutchTorque.SetParameters(1.f);

        // Initialise system states
        Eigen::Vector<float, 1> initLocked;
        initLocked << initEngineSpeed;
        this->m_LockedState.SetInitialConditions(initLocked);

        Eigen::Vector2f initUnlocked = {initEngineSpeed, initWheelSpeed};
        this->m_UnLockedState.SetInitialConditions(initUnlocked);
        this->m_Switch.SetIndex(0);
    };



    void Powertrain::Configure(
            const SimFramework::Signal<float>* inTorqueEngine,
            const SimFramework::Signal<float>* inTorqueWheel,
            const SimFramework::Signal<float>* inClutchEngagement)
    {
        // Vector inputs
        this->m_UnlockedInput.Configure({this->m_SignedClutchTorque.OutForce(), inTorqueEngine, inTorqueWheel});
        this->m_LockedInput.Configure({inTorqueEngine, inTorqueWheel});

        // State space
        this->m_UnLockedState.Configure(this->m_UnlockedInput.OutSignal());
        this->m_LockedState.Configure(this->m_LockedInput.OutSignal());

        // Switch
        this->m_Switch.Configure({this->m_UnLockedState.OutSignal(), this->m_LockedState.OutSignal()}, 0);

        // Speed mask outputs
        this->m_StateMask.Configure(this->m_Switch.OutSignal());
        this->m_RelativeSpeed.Configure({this->m_StateMask.OutSignal(0), this->m_StateMask.OutSignal(1)}, {1.f, -1.f});

        // Lock state manager
        this->m_CrossingDetect.Configure(this->m_RelativeSpeed.OutSignal());
        this->m_LockStateController.Configure(this->OutSpeed1(), inTorqueEngine, inTorqueWheel, this->m_CrossingDetect.OutCrossing(), this->m_ClutchTorqueCapacity.OutSignal(), this);

        // Clutch
        this->m_ClutchTorqueCapacity.Configure(inClutchEngagement);
        this->m_SignedClutchTorque.Configure(this->m_RelativeSpeed.OutSignal(), this->m_ClutchTorqueCapacity.OutSignal());
    }


    const SimFramework::Signal<float>* Powertrain::OutSpeed1() const
    {
        return this->m_StateMask.OutSignal(0);
    };

    const SimFramework::Signal<float>* Powertrain::OutSpeed2() const
    {
        return this->m_StateMask.OutSignal(1);
    };

    const SimFramework::Signal<int>* Powertrain::OutEngagement() const
    {
        return this->m_LockStateController.OutState();
    };


    SimFramework::BlockList Powertrain::Blocks()
    {
        // Construct system
        return {{},
                {&(this->m_UnLockedState), &(this->m_LockedState)},
                {&(this->m_UnlockedInput), &(this->m_LockedInput), &(this->m_Switch), &(this->m_StateMask), &(this->m_RelativeSpeed), &(this->m_CrossingDetect), &(this->m_ClutchTorqueCapacity), &(this->m_SignedClutchTorque)},                {&(this->m_LockStateController)},
                {}};
    };

    void Powertrain::TransitionState(ELockupClutchState newState)
    {
        switch (newState)
        {
            case ELockupClutchState::e_Locked:
            {
                // Change switch indices
                this->m_Switch.SetIndex(1);

                // When locking, state speed is engine speed
                Eigen::Vector<float, 1> initialConditions;
                initialConditions << this->m_StateMask.OutSignal(0)->Read();
                this->m_LockedState.SetInitialConditions(initialConditions);
                this->m_LockedState.Initialise(0.f);

                break;
            }

            case ELockupClutchState::e_Unlocked:
            {
                // Change switch indices
                this->m_Switch.SetIndex(0);

                // Initialise state space models
                Eigen::Vector2f initialConditions = {this->m_StateMask.OutSignal(0)->Read(), this->m_StateMask.OutSignal(1)->Read()};
                this->m_UnLockedState.SetInitialConditions(initialConditions);
                this->m_UnLockedState.Initialise(0.f);

                break;
            }
        }
    }

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Powertrain::LogSignals()
    {
        return {{"Clutch Lock State", this->m_LockStateController.OutState()},
                {"Clutch Torque Capacity", this->m_ClutchTorqueCapacity.OutSignal()},
                {"Clutch Signed Torque", this->m_SignedClutchTorque.OutForce()},
                {"Clutch Speed Cross", this->m_CrossingDetect.OutCrossing()},
                {"Clutch Speed 1", this->OutSpeed1()},
                {"Clutch Speed 2", this->OutSpeed2()}};
    };


    void Powertrain::SetGearRatio()
    {
        float G = this->m_Ratios[this->m_GearIndex];

        this->UpdateSSMatrices(G, this->b_e, this->b_w, this->I_e, this->I_w);
    };

    void Powertrain::UpdateSSMatrices(float G, float b_e, float b_w, float I_e, float I_w)
    {
        // Set up state space matrices for locked state
        float I_eff = I_w + G * G * I_e;

        Eigen::Matrix<float, 1, 1> lockedA;
        lockedA << - (G * G * b_e + b_w) / I_eff;

        Eigen::Matrix<float, 1, 2> lockedB;
        lockedB << (G * G) / I_eff, - G / I_eff;

        Eigen::Matrix<float, 2, 1> lockedC;
        lockedC << 1.f, 1.f / G;

        Eigen::Matrix<float, 2, 2> lockedD = Eigen::Matrix<float, 2, 2>::Zero();

        this->m_LockedState.SetMatrices(lockedA, lockedB, lockedC, lockedD);

        // Set up state space matrices for unlocked state
        Eigen::Matrix<float, 2, 2> unlockedA;
        unlockedA << - b_e / I_e, 0.f, 0.f, - b_w / I_w;

        Eigen::Matrix<float, 2, 3> unlockedB;
        unlockedB << - 1.f / I_e, 1.f / I_e, 0.f, G / I_w, 0, - 1.f / I_w;

        Eigen::Matrix<float, 2, 2> unlockedC;
        unlockedC << 1.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<float, 2, 3> unlockedD = Eigen::Matrix<float, 2, 3>::Zero();

        this->m_UnLockedState.SetMatrices(unlockedA, unlockedB, unlockedC, unlockedD);
    };

}; // namespace Models