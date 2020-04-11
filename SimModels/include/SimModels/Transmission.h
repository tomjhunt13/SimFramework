#ifndef FRAMEWORK_TRANSMISSION_H
#define FRAMEWORK_TRANSMISSION_H

#include "Eigen/Dense"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"
#include "SimModels/VehicleComponents.h"

namespace Models {

    class Transmission : public SimFramework::Subsystem {
    public:
        void SetParameters(std::vector<float> gearRatios);

        bool ShiftUp();
        bool ShiftDown();
        void Configure(
                const SimFramework::Signal<float>* inClutchSpeed,
                const SimFramework::Signal<float>* inTyreTorque);

        const SimFramework::Signal<float>* OutWheelSpeed() const;
        const SimFramework::Signal<float>* OutClutchTorque() const;
        const SimFramework::Signal<int>* OutGearIndex() const;

        SimFramework::BlockList Blocks() override;
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;

    private:
        void SetGearRatio();

        // Parameters
        std::vector<float> m_Ratios;
        int m_GearIndex;

        // Gain blocks
        SimFramework::Gain<float, float, float> m_SpeedGain;
        SimFramework::Gain<float, float, float> m_TorqueGain;

        // Gear index monitoring
        SimFramework::Input<int> m_InGearIndex;
    };

}; // namespace Models


#endif //FRAMEWORK_TRANSMISSION_H
