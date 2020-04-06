#ifndef FRAMEWORK_TRANSMISSION_H
#define FRAMEWORK_TRANSMISSION_H

#include "Eigen/Dense"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"
#include "SimModels/VehicleComponents.h"

namespace Models {

    class Transmission : public SimFramework::Subsystem {
    public:
        void SetParameters(
                std::vector<float> gearRatios = {0.07, 0.14, 0.23, 0.32, 0.41, 0.5}, float effectiveInertia = 1.f,
                float Mu=0.9, float R=0.15, float D=0.01, float maxBrakePressure=50000, int N=2);

        bool ShiftUp();
        bool ShiftDown();
        int CurrentGear() const;

        void Configure(
                const SimFramework::Signal<float>* inClutchTorque,
                const SimFramework::Signal<float>* inTyreTorque,
                const SimFramework::Signal<float>* inBrakePressure);

        const SimFramework::Signal<float>* OutClutchSpeed() const;
        const SimFramework::Signal<float>* OutTyreSpeed() const;

        SimFramework::BlockList Blocks() override;
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;

    private:
        void SetGearRatio();

        // Parameters
        std::vector<float> m_Ratios;
        int m_GearIndex;
        float m_EffectiveInertia;


        // Blocks
        DiscBrake m_DiscBrake;
        SimFramework::Vectorise<float, Eigen::Vector3f> m_TorqueVector;
        SimFramework::StateSpace<Eigen::Vector3f, Eigen::Vector2f, 3, 1, 2> m_States;
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_StateMask;
    };

}; // namespace Models


#endif //FRAMEWORK_TRANSMISSION_H
