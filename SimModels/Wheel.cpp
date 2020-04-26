#include "SimModels/Wheel.h"

namespace Models {


    Tyre::Tyre(std::string name) : Function(name) {};


    void Tyre::Configure(const SimFramework::Signal<float> *inRotationalSpeed,
                         const SimFramework::Signal<float> *inLinearSpeed)
    {
        this->m_RotationalSpeed = inRotationalSpeed;
        this->m_LinearSpeed = inLinearSpeed;

        this->SetParameters();
    }

    void Tyre::SetParameters(float radius, float Fz, float D, float C, float B, float E)
    {
        // TODO: Fz could be an input signal
        this->radius = radius;
        this->Fz = Fz;
        this->B = B;
        this->C = C;
        this->D = D;
        this->E = E;
    }

    const SimFramework::Signal<float>*  Tyre::OutForce() const
    {
        return &(this->m_Force);
    };

    const SimFramework::Signal<float>*  Tyre::OutTorque() const
    {
        return &(this->m_Torque);
    };

    std::vector<const SimFramework::SignalBase*> Tyre::InputSignals() const
    {
        return {&(this->m_RotationalSpeed), &(this->m_LinearSpeed)};
    };

    std::vector<const SimFramework::SignalBase*> Tyre::OutputSignals() const
    {
        return {&(this->m_Force), &(this->m_Torque)};
    };

    void Tyre::Update()
    {
        // Input signals
        float V = this->m_LinearSpeed->Read();
        float omega = this->m_RotationalSpeed->Read();

        // Calculate slip ratio
        float V_abs = std::abs(V);
        float V_sx = omega * this->radius - V;
        float k;

        if (std::abs(V) <= this->V_threshold)
        {
            k = (2.f * V_sx) / (V_threshold + (V * V) / V_threshold);
        }
        else
        {
            k = (V_sx) / V_abs;
        }

        float Fx = this->Fz * this->D * std::sin(this->C * std::atan(this->B * k - this->E * (this->B * k - std::atan(this->B * k))));

        // Write result to output signals
        this->m_Force.Write(Fx);
        this->m_Torque.Write(Fx * this->radius);
    };



    void Wheel::SetParameters(float peakBreakForce)
    {
        this->m_Brake.SetParameters(peakBreakForce);
    };

    void Wheel::Configure(
            const SimFramework::Signal<float>* inBrake,
            const SimFramework::Signal<float>* inRotationalSpeed,
            const SimFramework::Signal<float>* inLinearSpeed)
    {
        this->m_Tyre.Configure(inRotationalSpeed, inLinearSpeed);
        this->m_Brake.Configure(inRotationalSpeed, inBrake);
        this->m_Sum.Configure({this->m_Tyre.OutTorque(), this->m_Brake.OutForce()}, {1.f, 1.f});
    };

    const SimFramework::Signal<float>* Wheel::OutForce() const
    {
        return this->m_Tyre.OutForce();
    };

    const SimFramework::Signal<float>* Wheel::OutTorque() const
    {
        return this->m_Sum.OutSignal();
    };

    SimFramework::BlockList Wheel::Blocks()
    {
        return {{},
                {},
                {&(this->m_Tyre), &(this->m_Sum), &(this->m_Brake)},
                {},
                {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Wheel::LogSignals()
    {
        return {};
    };

}; // namespace Models