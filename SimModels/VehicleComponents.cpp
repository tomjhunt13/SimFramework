#include "SimModels/VehicleComponents.h"


namespace Models {


    LinearTrigger::LinearTrigger(std::string name) : SimFramework::TriggerFunction(name) {};

    void LinearTrigger::SetParameters(float defaultValue, float t_end)
    {
        this->m_Default = defaultValue;
        this->t_end = 1.f;
    }

    float LinearTrigger::Evaluate(float t) {
        return 1.f * (this->t_end - t) / (this->t_end);
    }


    CoulombFriction::CoulombFriction(std::string name) : SimFramework::Function(name) {};

    void CoulombFriction::SetParameters(float mu)
    {
        this->mu = mu;
    };

    void CoulombFriction::Configure(const SimFramework::Signal<float>* inVelocity, const SimFramework::Signal<float>* inNormalForce)
    {
        this->m_Velocity = inVelocity;
        this->m_NormalForce = inNormalForce;
    };

    const SimFramework::Signal<float>* CoulombFriction::OutForce() const
    {
        return &(this->m_Force);
    };

    std::vector<const SimFramework::SignalBase*> CoulombFriction::InputSignals() const
    {
        return {this->m_Velocity, this->m_NormalForce};
    };

    std::vector<const SimFramework::SignalBase*> CoulombFriction::OutputSignals() const
    {
        return {&(this->m_Force)};
    };

    void CoulombFriction::Update()
    {
        // Read inputs
        float speed = this->m_Velocity->Read();
        float normalForce = this->m_NormalForce->Read();

        // Calculate output force
        float output = this->mu * normalForce;

        // Write output
        if (speed < 0)
        {
            output *= -1.f;
        };

        this->m_Force.Write(output);
    };


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


    void UnitConversions::Configure(
            const SimFramework::Signal<float>* inEngineSpeedRadiansPerSecond,
            const SimFramework::Signal<float>* inCarSpeedMetresPerSecond,
            const SimFramework::Signal<float>* inCarDisplacementMetres,
            const SimFramework::Signal<float>* inFuelFlowRateGramsPerSecond,
            const SimFramework::Signal<float>* inCumulativeFuelUsageGrams)
    {
        this->m_EngineSpeedRadiansPerSecond = inEngineSpeedRadiansPerSecond;
        this->m_CarSpeedMetresPerSecond = inCarSpeedMetresPerSecond;
        this->m_CarDisplacementMetres = inCarDisplacementMetres;
        this->m_FuelFlowRateGramsPerSecond = inFuelFlowRateGramsPerSecond;
        this->m_CumulativeFuelUsageGrams = inCumulativeFuelUsageGrams;
    };

    void UnitConversions::SetParameters(EUnitSystem unitSystem)
    {
        this->m_UnitSystem = unitSystem;
    };

    const SimFramework::Signal<float>*  UnitConversions::OutEngineSpeed() const
    {
        return &(this->m_OutEngineSpeed);
    };

    const SimFramework::Signal<float>*  UnitConversions::OutCarSpeed() const
    {
        return &(this->m_OutCarSpeed);
    };

    const SimFramework::Signal<float>*  UnitConversions::OutInstantFuelEfficiency() const
    {
        return &(this->m_OutInstantFuelEfficiency);
    };

    const SimFramework::Signal<float>*  UnitConversions::OutAverageFuelEfficiency() const
    {
        return &(this->m_OutAverageFuelEfficiency);
    };

    std::vector<const SimFramework::SignalBase*> InputSignals() const;
    std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
    void Update() override;

}; // namespace Models