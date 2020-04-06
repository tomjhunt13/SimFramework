#ifndef FRAMEWORK_TRANSMISSION_H
#define FRAMEWORK_TRANSMISSION_H

#include "Eigen/Dense"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"
#include "SimModels/VehicleComponents.h"

namespace Models {

    class Transmission : public SimFramework::Subsystem {
    public:
        void SetParameters(std::vector<float> gearRatios, float effectiveInertia = 1.f);

        bool ShiftUp();
        bool ShiftDown();
        void Configure(
                const SimFramework::Signal<float>* inClutchTorque,
                const SimFramework::Signal<float>* inTyreTorque);

        const SimFramework::Signal<float>* OutClutchSpeed() const;
        const SimFramework::Signal<float>* OutTyreSpeed() const;
        const SimFramework::Signal<int>* OutGearIndex() const;

        SimFramework::BlockList Blocks() override;
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;

    private:
        void SetGearRatio();

        // Parameters
        std::vector<float> m_Ratios;
        int m_GearIndex;
        float m_EffectiveInertia;

        // Blocks
        SimFramework::Vectorise<float, Eigen::Vector2f> m_TorqueVector;
        SimFramework::StateSpace<Eigen::Vector2f, Eigen::Vector2f, 2, 1, 2> m_States;
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_StateMask;

        // Gear index monitoring
        SimFramework::Input<int> m_InGearIndex;
    };

}; // namespace Models


#endif //FRAMEWORK_TRANSMISSION_H
