#include "SimModels/Transmission.h"


namespace Models {

    void Transmission::SetParameters(
            std::vector<float> gearRatios, float effectiveInertia,
            float Mu, float R, float D, float maxBrakePressure, int N)
    {
        this->m_Ratios = gearRatios;
        this->m_EffectiveInertia = effectiveInertia;
        this->m_DiscBrake.SetParameters(Mu, R, D, maxBrakePressure, N);

        // Initialise dynamic variables and initial conditions
        this->m_GearIndex = 0;
        this->SetGearRatio();

        Eigen::Vector<float, 1> initialConditions;
        initialConditions << 0.f;
        this->m_States.SetInitialConditions(initialConditions);
    }

    void Transmission::Configure(
            const SimFramework::Signal<float>* inClutchTorque,
            const SimFramework::Signal<float>* inTyreTorque,
            const SimFramework::Signal<float>* inBrakePressure)
    {
        // Configure blocks
        this->m_DiscBrake.Configure(inBrakePressure, this->OutTyreSpeed());
        this->m_TorqueVector.Configure({inClutchTorque, inTyreTorque, this->m_DiscBrake.OutTorque()});
        this->m_States.Configure(this->m_TorqueVector.OutSignal());
        this->m_StateMask.Configure(this->m_States.OutSignal());
    };

    const SimFramework::Signal<float>* Transmission::OutClutchSpeed() const
    {
        return this->m_StateMask.OutSignal(0);
    };

    const SimFramework::Signal<float>* Transmission::OutTyreSpeed() const
    {
        return this->m_StateMask.OutSignal(1);
    };


    SimFramework::BlockList Transmission::Blocks()
    {
        return {{},
                {&(this->m_States)},
                {&(this->m_TorqueVector), &(this->m_StateMask), &(this->m_DiscBrake)},
                {},
                {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Transmission::LogSignals()
    {
        return {{"Transmission Clutch Speed, Transmission Wheel Speed", this->m_States.OutSignal()},
                {"Transmission Clutch Torque, Transmission Tyre Torque, Transmission Brake Torque", this->m_TorqueVector.OutSignal()}};
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

        return true;
    };

    void Transmission::SetGearRatio()
    {
        Eigen::Matrix<float, 1, 1> A;
        A << 0.f;

        Eigen::Matrix<float, 1, 3> B;
        B << 1.f / this->m_EffectiveInertia, - this->m_Ratios[this->m_GearIndex] / this->m_EffectiveInertia, this->m_Ratios[this->m_GearIndex] / this->m_EffectiveInertia;

        Eigen::Matrix<float, 2, 1> C;
        C << 1.f, this->m_Ratios[this->m_GearIndex];

        Eigen::Matrix<float, 2, 3> D;
        D << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;

        this->m_States.SetMatrices(A, B, C, D);
    }

    int Transmission::CurrentGear() const
    {
        return this->m_GearIndex + 1;
    }
}