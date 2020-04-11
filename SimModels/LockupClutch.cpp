#include "SimModels/LockupClutch.h"

namespace Models {

    TransmittedTorque::TransmittedTorque(std::string name) : SimFramework::Function(name) {};

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



    void LockupClutch::SetParameters(std::string engineJSON, float initialSpeed, std::vector<float> gearRatios, float b_e, float b_t, float I_e, float I_t, float maxClutchTorque)
    {
        // Set up parameters
        this->b_e = b_e;
        this->b_t = b_t;
        this->I_e = I_e;
        this->I_t = I_t;

        // Set up gear ratios
        std::vector<float> ratios(1 + gearRatios.size());
        ratios[0] = 0;
        for (int i = 0; i < gearRatios.size(); i++)
        {
            ratios[i+1] = gearRatios[i];
        }
        this->m_Ratios = ratios;
        this->m_GearIndex = 0;
        this->m_InGearIndex.WriteValue(this->m_GearIndex);
        this->SetGearRatio(0.f);

        // Engine maps
        SimFramework::Table3D torqueTable = SimFramework::ReadTableJSON(engineJSON, "speed", "throttle", "torque");
        this->m_TorqueMap.SetTable(torqueTable);

        SimFramework::Table3D fuelTable = SimFramework::ReadTableJSON(engineJSON, "speed", "throttle", "fuel");
        this->m_FuelMap.SetTable(fuelTable);

        // Clutch
        this->m_MaxClutchTorque.SetParameters(maxClutchTorque);

        // Initialise states
        Eigen::Vector<float, 2> unlockedInit = {200.f, 0.f};
        this->m_LockedState.SetInitialConditions(Eigen::Vector<float, 1>::Zero());
        this->m_UnLockedState.SetInitialConditions(unlockedInit);
        this->m_FuelIntegrator.SetInitialConditions(Eigen::Vector<float, 1>::Zero());

        // Switches
        this->m_EngineSpeedSwitch.SetIndex(0);
        this->m_WheelSpeedSwitch.SetIndex(0);

        // Lock state manager
        this->m_CrossingDetect.SetParameters(0.f, false);
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
        this->m_RelativeSpeed.Configure({this->m_UnlockedMask.OutSignal(1), this->m_UnlockedMask.OutSignal(0)}, {1.f, -1.f});
        this->m_MaxClutchTorque.Configure(this->m_RelativeSpeed.OutSignal(), inClutchPosition);

        // State space
        this->m_UnLockedState.Configure(this->m_UnlockedInput.OutSignal());
        this->m_LockedState.Configure(this->m_LockedInput.OutSignal());
        this->m_FuelIntegrator.Configure(this->m_FuelMap.OutSignal());

        // Vector inputs
        this->m_UnlockedInput.Configure({this->m_TorqueMap.OutSignal(), this->m_MaxClutchTorque.OutForce(), inTyreTorque});
        this->m_LockedInput.Configure({this->m_TorqueMap.OutSignal(), inTyreTorque});

        // Mask outputs
        this->m_UnlockedMask.Configure(this->m_UnLockedState.OutSignal());
        this->m_LockedMask.Configure(this->m_LockedState.OutSignal());
        this->m_FuelUsageMask.Configure(this->m_FuelIntegrator.OutSignal());

        // Switches
        this->m_EngineSpeedSwitch.Configure({this->m_UnlockedMask.OutSignal(1), this->m_LockedMask.OutSignal(1)}, 0);
        this->m_WheelSpeedSwitch.Configure({this->m_UnlockedMask.OutSignal(0), this->m_LockedMask.OutSignal(0)}, 0);

        // Lock state manager
        this->m_CrossingDetect.Configure(this->m_RelativeSpeed.OutSignal());
        this->m_CrossingDetect.SetParameters(0, false);
        this->m_LockStateController.Configure(this->m_CrossingDetect.OutCrossing(), this->m_TransmittedTorque.OutTorque(), this->m_MaxClutchTorque.OutForce(), this);

    }

    bool LockupClutch::ShiftUp()
    {
        // Ignore if in top gear
        if (this->m_GearIndex == this->m_Ratios.size() - 1)
        {
            return false;
        }

        // Else increment gear
        this->m_GearIndex += 1;

        // Change gear
        this->SetGearRatio(this->m_Ratios[this->m_GearIndex]);

        // Update input block
        this->m_InGearIndex.WriteValue(this->m_GearIndex);

        return true;
    };

    bool LockupClutch::ShiftDown()
    {
        // Ignore if in bottom gear
        if (this->m_GearIndex == 0)
        {
            return false;
        }

        // Else decrement gear
        this->m_GearIndex -= 1;

        // Change gear
        this->SetGearRatio(this->m_Ratios[this->m_GearIndex]);

        // Update input block
        this->m_InGearIndex.WriteValue(this->m_GearIndex);

        return true;
    };

    const SimFramework::Signal<float>* LockupClutch::OutEngineSpeed() const
    {
        return this->m_EngineSpeedSwitch.OutSignal();
    };

    const SimFramework::Signal<float>* LockupClutch::OutWheelSpeed() const
    {
        return this->m_WheelSpeedSwitch.OutSignal();
    };
    const SimFramework::Signal<int>* LockupClutch::OutGearIndex() const
    {
        return this->m_InGearIndex.OutSignal();
    };

    const SimFramework::Signal<float>* LockupClutch::OutFuelRate() const
    {
        return this->m_FuelMap.OutSignal();
    };

    const SimFramework::Signal<float>* LockupClutch::OutFuelCumulative() const
    {
        return this->m_FuelUsageMask.OutSignal(0);
    };

    SimFramework::BlockList LockupClutch::Blocks()
    {
        // Construct system
        return {{&(this->m_InGearIndex)},
                {&(this->m_UnLockedState), &(this->m_LockedState), &(this->m_FuelIntegrator)},
                {&(this->m_TorqueMap), &(this->m_FuelMap), &(this->m_TransmittedTorque), &(this->m_RelativeSpeed), &(this->m_MaxClutchTorque), &(this->m_UnlockedInput), &(this->m_LockedInput), &(this->m_UnlockedMask), &(this->m_LockedMask), &(this->m_FuelUsageMask), &(this->m_EngineSpeedSwitch), &(this->m_WheelSpeedSwitch), &(this->m_CrossingDetect)},
                {&(this->m_LockStateController)},
                {}};
    };

    void LockupClutch::TransitionState(ELockupClutchState newState)
    {


        switch (newState)
        {
            case ELockupClutchState::e_Locked:
            {
                // Change switch indices
                this->m_WheelSpeedSwitch.SetIndex(1);
                this->m_EngineSpeedSwitch.SetIndex(1);

                // Initialise state space models
                Eigen::Vector<float, 1> initialConditions;
                initialConditions << this->m_WheelSpeedSwitch.OutSignal()->Read();
                this->m_LockedState.SetInitialConditions(initialConditions);
                this->m_LockedState.Initialise(0.f);

                break;
            }

            case ELockupClutchState::e_Unlocked:
            {
                // Change switch indices
                this->m_WheelSpeedSwitch.SetIndex(0);
                this->m_EngineSpeedSwitch.SetIndex(0);

                // Initialise state space models
                Eigen::Vector2f initialConditions = {this->m_WheelSpeedSwitch.OutSignal()->Read(), this->m_WheelSpeedSwitch.OutSignal()->Read()};
                this->m_UnLockedState.SetInitialConditions(initialConditions);
                this->m_UnLockedState.Initialise(0.f);

                break;
            }
        }
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

        // Set parameters for clutch transmission
        this->m_TransmittedTorque.SetParameters({G, this->b_e, this->b_t, this->I_e, this->I_t});

    };

}; // namespace Models