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

    void UnitConversions::SetParameters(EUnitSystem unitSystem, float fuelDensity)
    {
        this->m_UnitSystem = unitSystem;
        this->m_FuelDensity= fuelDensity;
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

    std::vector<const SimFramework::SignalBase*> UnitConversions::InputSignals() const
    {
        return {this->m_EngineSpeedRadiansPerSecond, this->m_CarSpeedMetresPerSecond, this->m_CarDisplacementMetres, this->m_FuelFlowRateGramsPerSecond, this->m_CumulativeFuelUsageGrams};
    };

    std::vector<const SimFramework::SignalBase*> UnitConversions::OutputSignals() const
    {
        return {this->OutEngineSpeed(), this->OutCarSpeed(), this->OutInstantFuelEfficiency(), this->OutAverageFuelEfficiency()};
    };

    void UnitConversions::Update()
    {
        // Read inputs
        float engineSpeedRadians = this->m_EngineSpeedRadiansPerSecond->Read();
        float carSpeedMetres = this->m_CarSpeedMetresPerSecond->Read();
        float carDisplacementMetres = this->m_CarDisplacementMetres->Read();
        float fuelFlowGrams = this->m_FuelFlowRateGramsPerSecond->Read();
        float fuelCumulativeGrams = this->m_CumulativeFuelUsageGrams->Read();

        // Compute conversions
        float fuelFlowCentimetresCubed = SimFramework::MassToVolume(fuelFlowGrams, this->m_FuelDensity);
        float fuelCumulativeCentimetresCubed = SimFramework::MassToVolume(fuelCumulativeGrams, this->m_FuelDensity);
        float outEngineSpeed = SimFramework::RadiansPerSecondToRPM(engineSpeedRadians);
        float outCarSpeed;
        float outInstantFuelEff;
        float outAvgFuelEff;

        switch (this->m_UnitSystem)
        {
            case EUnitSystem::e_Metric:
            {
                outCarSpeed = SimFramework::MetresPerSecondToKPH(carSpeedMetres);

                outInstantFuelEff =  100.f * (SimFramework::CentimetresCubedToLitres(fuelFlowCentimetresCubed) / (SimFramework::MetresToKilometers(carSpeedMetres)));

                outAvgFuelEff =  100.f * (SimFramework::CentimetresCubedToLitres(fuelCumulativeCentimetresCubed) / (SimFramework::MetresToKilometers(carDisplacementMetres)));

                break;
            }

            case EUnitSystem::e_Imperial:
            {
                outCarSpeed = SimFramework::MetresPerSecondToMPH(carSpeedMetres);

                outInstantFuelEff = SimFramework::MetresToMiles(carSpeedMetres) / SimFramework::CentimetresCubedToGallons(fuelFlowCentimetresCubed);

                outAvgFuelEff = SimFramework::MetresToMiles(carDisplacementMetres) / SimFramework::CentimetresCubedToGallons(fuelCumulativeCentimetresCubed);

                break;
            }
        }

        // Write outputs
        this->m_OutEngineSpeed.Write(outEngineSpeed);
        this->m_OutCarSpeed.Write(outCarSpeed);
        this->m_OutInstantFuelEfficiency.Write(outInstantFuelEff);
        this->m_OutAverageFuelEfficiency.Write(outAvgFuelEff);
    };

}; // namespace Models