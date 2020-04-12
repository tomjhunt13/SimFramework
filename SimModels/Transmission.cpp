#include "SimModels/Transmission.h"

namespace Models {

    void Transmission::SetParameters(std::vector<float> gearRatios)
    {

        // Set up gear ratios
        std::vector<float> ratios(1 + gearRatios.size());
        ratios[0] = gearRatios[0];
        for (int i = 0; i < gearRatios.size(); i++)
        {
            ratios[i+1] = gearRatios[i];
        }
        this->m_Ratios = ratios;

        // Initialise dynamic variables and initial conditions
        this->m_GearIndex = 0;
        this->SetGearRatio();
        this->m_InGearIndex.WriteValue(this->m_GearIndex);
    }

    void Transmission::Configure(
            const SimFramework::Signal<float>* inClutchSpeed,
            const SimFramework::Signal<float>* inTyreTorque)
    {
        // Configure blocks
        this->m_SpeedGain.Configure(inClutchSpeed);
        this->m_TorqueGain.Configure(inTyreTorque);
        this->m_InGearIndex.Configure(0);
    };

    const SimFramework::Signal<float>* Transmission::OutWheelSpeed() const
    {
        return this->m_SpeedGain.OutSignal();
    };

    const SimFramework::Signal<float>* Transmission::OutClutchTorque() const
    {
        return this->m_TorqueGain.OutSignal();
    };

    const SimFramework::Signal<int>* Transmission::OutGearIndex() const
    {
        return this->m_InGearIndex.OutSignal();
    };

    SimFramework::BlockList Transmission::Blocks()
    {
        return {{&(this->m_InGearIndex)},
                {},
                {&(this->m_TorqueGain), &(this->m_SpeedGain)},
                {},
                {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Transmission::LogSignals()
    {
        return {};
    };

    bool Transmission::ShiftUp()
    {
        // Ignore if in top gear
        if (this->m_GearIndex == this->m_Ratios.size() - 1)
        {
            return false;
        }

        // Else increment gear
        this->m_GearIndex += 1;

        // Change gear
        this->SetGearRatio();

        // Update input block
        this->m_InGearIndex.WriteValue(this->m_GearIndex);

        return true;
    };

    bool Transmission::ShiftDown()
    {
        // Ignore if in bottom gear
        if (this->m_GearIndex == 0)
        {
            return false;
        }

        // Else decrement gear
        this->m_GearIndex -= 1;

        // Change gear
        this->SetGearRatio();

        // Update input block
        this->m_InGearIndex.WriteValue(this->m_GearIndex);

        return true;
    };

    void Transmission::SetGearRatio()
    {
        float G = this->m_Ratios[this->m_GearIndex];

        this->m_TorqueGain.SetGain(1.f / G);
        this->m_SpeedGain.SetGain(1.f / G);
    }
}