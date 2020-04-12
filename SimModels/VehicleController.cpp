#include "SimModels/VehicleController.h"


namespace Models {

    ClutchController::ClutchController(std::string name) :
            Function(name) {};

    void ClutchController::SetParameters(float engagementSpeed)
    {
        this->m_EngagementSpeed = engagementSpeed;
    }

    void ClutchController::Configure(
            const SimFramework::Signal<float>* inTransmissionSpeed,
            const SimFramework::Signal<float>* inThrottle,
            const SimFramework::Signal<int>* inGearIndex)
    {
        this->m_InTransmissionSpeed = inTransmissionSpeed;
        this->m_InThrottle = inThrottle;
        this->m_InGearIndex = inGearIndex;
    };

    const SimFramework::Signal<float>* ClutchController::OutEngagement() const
    {
        return &(this->m_OutEngagement);
    };


    std::vector<const SimFramework::SignalBase*> ClutchController::InputSignals() const
    {
        return {this->m_InTransmissionSpeed};
    };

    std::vector<const SimFramework::SignalBase*> ClutchController::OutputSignals() const
    {
        return {&(this->m_OutEngagement)};
    };

    void ClutchController::Update()
    {
        float throttle = this->m_InThrottle->Read();
        float speed = this->m_InTransmissionSpeed->Read();
        int gear = this->m_InGearIndex->Read();

        this->m_OutEngagement.Write(this->Evaluate(speed, throttle, gear));
    };

    float ClutchController::Evaluate(float transmissionSpeed, float throttle, int gear)
    {
//        // If in neutral then release clutch
//        if (gear == 0)
//        {
//            return 0.f;
//        };

        // If clutch speed is above engagement threshold then fully engage clutch
        if (transmissionSpeed > this->m_EngagementSpeed)
        {
            return 1.f;
        };

//        // As a precaution: if transmission speed is negative clutch should be released
//        if (transmissionSpeed < 0.f)
//        {
//            return 0.f;
//        };

        // If throttle is above threshold then assume clutch should be trying to engage
        if (throttle > this->m_ThrottleThreshold)
        {
            return 0.5 + 0.9 * (transmissionSpeed / this->m_EngagementSpeed);
        }

        // Else speed is in range [0, engagement speed] and throttle below threshold so release clutch
        return 0.f;

    };



    void VehicleController::SetParameters(float clutchLagTime, float clutchEngagementSpeed)
    {
        this->m_GearChangeTrigger.SetParameters(0.f, clutchLagTime);
        this->m_ClutchController.SetParameters(clutchEngagementSpeed);
    }

    void VehicleController::Configure(
            const SimFramework::Signal<float>* inTransmissionSpeed,
            const SimFramework::Signal<float>* inThrottle,
            const SimFramework::Signal<int>* inGearIndex)
    {
        // Constants
        this->m_ConstZero.Configure(0.f);

        // Configure clutch blocks
        this->m_ClutchController.Configure(inTransmissionSpeed, inThrottle, inGearIndex);
        this->m_BlendClutch.Configure(this->m_ClutchController.OutEngagement(), this->m_ConstZero.OutSignal(), this->m_GearChangeTrigger.OutSignal());

        // Configure throttle blocks
        this->m_BlendThrottle.Configure(inThrottle, this->m_ConstZero.OutSignal(), this->m_GearChangeTrigger.OutSignal());
    };

    const SimFramework::Signal<float>* VehicleController::OutAugmentedThrottle() const
    {
        return this->m_BlendThrottle.OutSignal();
    };

    const SimFramework::Signal<float>* VehicleController::OutClutchStiffness() const
    {
        return this->m_BlendClutch.OutSignal();
    };

    void VehicleController::Trigger()
    {
        this->m_GearChangeTrigger.Trigger();
    };

    SimFramework::BlockList VehicleController::Blocks()
    {
        return {{&(this->m_GearChangeTrigger), &(this->m_ConstZero)},
                {},
                {&(this->m_ClutchController), &(this->m_BlendThrottle), &(this->m_BlendClutch)},
                {},
                {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > VehicleController::LogSignals()
    {
        return {{"Clutch Stiffness", this->OutClutchStiffness()},
                {"Augmented Throttle", this->OutAugmentedThrottle()}};
    };

}; // namespace Models