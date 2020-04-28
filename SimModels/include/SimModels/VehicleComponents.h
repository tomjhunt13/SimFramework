#ifndef FRAMEWORK_VEHICLECOMPONENTS_H
#define FRAMEWORK_VEHICLECOMPONENTS_H

#include <cmath>

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"
#include "SimFramework/Utilities.h"

namespace Models {

    class LinearTrigger : public SimFramework::TriggerFunction {
    public:
        LinearTrigger(std::string name = "Linear Trigger");
        void SetParameters(float defaultValue = 0.f, float t_end = 1.f);

        float Evaluate(float t);
    };


    class CoulombFriction : public SimFramework::Function
    {
    public:
        CoulombFriction(std::string name="Coulomb Friction");

        void SetParameters(float mu=1.f);
        void Configure(const SimFramework::Signal<float>* inVelocity, const SimFramework::Signal<float>* inNormalForce);
        const SimFramework::Signal<float>* OutForce() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:
        // Parameters
        float mu;

        // Signals
        const SimFramework::Signal<float>* m_Velocity;
        const SimFramework::Signal<float>* m_NormalForce;
        SimFramework::Signal<float> m_Force;
    };


    class Tyre : public SimFramework::Function
    {
    public:
        Tyre(std::string name = "Tyre");

        void Configure(const SimFramework::Signal<float>* inRotationalSpeed,
                       const SimFramework::Signal<float>* inLinearSpeed);

        const SimFramework::Signal<float>*  OutForce() const;
        const SimFramework::Signal<float>*  OutTorque() const;

        void SetParameters(float radius=0.2, float Fz=15000,  float D=1, float C=1.9, float B=10, float E=0.97);

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:
        // Parameters
        float radius;
        float Fz;
        float B;
        float C;
        float D;
        float E;
        float V_threshold = 0.001;

        // Signals
        const SimFramework::Signal<float>* m_RotationalSpeed;
        const SimFramework::Signal<float>* m_LinearSpeed;
        SimFramework::Signal<float> m_Force;
        SimFramework::Signal<float> m_Torque;
    };


    enum class EUnitSystem {
        e_Metric,
        e_Imperial
    };

    class UnitConversions : public SimFramework::Function
    {
    public:
        void Configure(const SimFramework::Signal<float>* inEngineSpeedRadiansPerSecond,
                       const SimFramework::Signal<float>* inCarSpeedMetresPerSecond,
                       const SimFramework::Signal<float>* inCarDisplacementMetres,
                       const SimFramework::Signal<float>* inFuelFlowRateGramsPerSecond,
                       const SimFramework::Signal<float>* inCumulativeFuelUsageGrams);

        void SetParameters(EUnitSystem unitSystem = EUnitSystem::e_Imperial, float fuelDensity = 0.7489);

        const SimFramework::Signal<float>*  OutEngineSpeed() const;
        const SimFramework::Signal<float>*  OutCarSpeed() const;
        const SimFramework::Signal<float>*  OutInstantFuelEfficiency() const;
        const SimFramework::Signal<float>*  OutAverageFuelEfficiency() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:

        // Parameters
        EUnitSystem m_UnitSystem;
        float m_FuelDensity;

        // Input Signals
        const SimFramework::Signal<float>* m_EngineSpeedRadiansPerSecond;
        const SimFramework::Signal<float>* m_CarSpeedMetresPerSecond;
        const SimFramework::Signal<float>* m_CarDisplacementMetres;
        const SimFramework::Signal<float>* m_FuelFlowRateGramsPerSecond;
        const SimFramework::Signal<float>* m_CumulativeFuelUsageGrams;

        // Output Signals
        SimFramework::Signal<float> m_OutEngineSpeed;
        SimFramework::Signal<float> m_OutCarSpeed;
        SimFramework::Signal<float> m_OutInstantFuelEfficiency;
        SimFramework::Signal<float> m_OutAverageFuelEfficiency;

    };

}; // namespace Models


#endif //FRAMEWORK_VEHICLECOMPONENTS_H
