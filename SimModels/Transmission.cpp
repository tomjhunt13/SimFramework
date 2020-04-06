#include "SimModels/Transmission.h"


namespace Models {

    void Transmission::SetParameters(std::vector<float> gearRatios, float effectiveInertia)
    {

        // Set up gear ratios
        std::vector<float> ratios(1 + gearRatios.size());
        ratios[0] = 0;
        for (int i = 0; i < gearRatios.size(); i++)
        {
            ratios[i+1] = gearRatios[i];
        }
        this->m_Ratios = ratios;


        this->m_EffectiveInertia = effectiveInertia;

        // Initialise dynamic variables and initial conditions
        this->m_GearIndex = 0;
        this->SetGearRatio();
        this->m_InGearIndex.WriteValue(this->m_GearIndex);

        this->m_States.SetInitialConditions(Eigen::Vector<float, 1>::Zero());
    }

    void Transmission::Configure(
            const SimFramework::Signal<float>* inClutchTorque,
            const SimFramework::Signal<float>* inTyreTorque)
    {
        // Configure blocks
        this->m_TorqueVector.Configure({inClutchTorque, inTyreTorque});
        this->m_States.Configure(this->m_TorqueVector.OutSignal());
        this->m_StateMask.Configure(this->m_States.OutSignal());
        this->m_InGearIndex.Configure(0);
    };

    const SimFramework::Signal<float>* Transmission::OutClutchSpeed() const
    {
        return this->m_StateMask.OutSignal(0);
    };

    const SimFramework::Signal<float>* Transmission::OutTyreSpeed() const
    {
        return this->m_StateMask.OutSignal(1);
    };

    const SimFramework::Signal<int>* Transmission::OutGearIndex() const
    {
        return this->m_InGearIndex.OutSignal();
    };



    SimFramework::BlockList Transmission::Blocks()
    {
        return {{&(this->m_InGearIndex)},
                {&(this->m_States)},
                {&(this->m_TorqueVector), &(this->m_StateMask)},
                {},
                {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Transmission::LogSignals()
    {
        return {{"Transmission Clutch Speed, Transmission Wheel Speed", this->m_States.OutSignal()},
                {"Transmission Clutch Torque, Transmission Tyre Torque", this->m_TorqueVector.OutSignal()},
                {"Transmission Gear Index", this->m_InGearIndex.OutSignal()}};
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
        Eigen::Matrix<float, 1, 1> A;
        A << 0.f;

        Eigen::Matrix<float, 1, 2> B;
        B << this->m_Ratios[this->m_GearIndex] / this->m_EffectiveInertia,  - 1 / this->m_EffectiveInertia;

        Eigen::Matrix<float, 2, 1> C;
        C << 1.f, this->m_Ratios[this->m_GearIndex];

        Eigen::Matrix<float, 2, 2> D;
        D << 0.f, 0.f, 0.f, 0.f;

        this->m_States.SetMatrices(A, B, C, D);
    }
}