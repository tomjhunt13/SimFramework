#ifndef FRAMEWORK_VEHICLECONTROLLER_H
#define FRAMEWORK_VEHICLECONTROLLER_H


#include "SimFramework/Framework.h"
#include "SimModels/VehicleComponents.h"

namespace Models {

    class ClutchController : public SimFramework::Function
    {
    public:
        ClutchController(float engagementSpeed = 50.f, std::string name = "Clutch Controller");

        void Configure(
                const SimFramework::Signal<float>* inTransmissionSpeed,
                const SimFramework::Signal<float>* inThrottle,
                const SimFramework::Signal<int>* inGearIndex);

        const SimFramework::Signal<float>* OutEngagement() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:
        float Evaluate(float transmissionSpeed, float throttle, int gear);

        // Parameters
        float m_EngagementSpeed;
        float m_ThrottleThreshold = 0.02;

        // Signals
        const SimFramework::Signal<float>* m_InTransmissionSpeed;
        const SimFramework::Signal<float>* m_InThrottle;
        const SimFramework::Signal<int>* m_InGearIndex;
        SimFramework::Signal<float> m_OutEngagement;
    };

    class VehicleController : public SimFramework::Subsystem
    {
    public:
        void SetParameters(float clutchLagTime=1.f, float clutchStiffness=1000.f);

        void Configure(
                const SimFramework::Signal<float>* inTransmissionSpeed,
                const SimFramework::Signal<float>* inThrottle,
                const SimFramework::Signal<int>* inGearIndex);

        const SimFramework::Signal<float>* OutAugmentedThrottle() const;
        const SimFramework::Signal<float>* OutClutchStiffness() const;

        void Trigger();

        SimFramework::BlockList Blocks() override;

        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;


    private:
        // Parameters
        float m_MaxClutchStiffness;

        // Blocks - Gear change
        LinearTrigger m_GearChangeTrigger;

        // Blocks - Throttle
        SimFramework::LinearBlend<float> m_BlendThrottle;

        // Blocks - Clutch
        ClutchController m_ClutchController;
        SimFramework::LinearBlend<float> m_BlendClutch;
        SimFramework::Gain<float, float, float> m_ClutchGain;

        // Blocks - Constants
        SimFramework::ConstantBlock<float> m_ConstZero;
    };

}; // namespace Models

#endif //FRAMEWORK_VEHICLECONTROLLER_H
