#ifndef FRAMEWORK_VEHICLECOMPONENTS_H
#define FRAMEWORK_VEHICLECOMPONENTS_H

#include <cmath>
#include "SimFramework/Utilities.h"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

namespace Models {

    class LinearTrigger : public SimFramework::TriggerFunction {
    public:
        LinearTrigger(std::string name = "Linear Trigger");
        void SetParameters(float defaultValue = 0.f, float t_end = 1.f);

        float Evaluate(float t);
    };


    class AeroDrag : public SimFramework::Function
    {
    public:
        AeroDrag(std::string name="Aero Drag");

        void SetParameters(float Cd=0.3, float A=2.5, float rho=1.225);
        void Configure(const SimFramework::Signal<float>* inSpeed);
        const SimFramework::Signal<float>* OutForce() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:
        // Parameters
        float Cd;
        float A;
        float rho;

        // Signals
        const SimFramework::Signal<float>* m_Speed;
        SimFramework::Signal<float> m_Force;
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


    class Gravity : public SimFramework::Function
    {
    public:
        Gravity(std::string name="Gravity");

        void SetParameters(float mass=1000.f);
        void Configure(const SimFramework::Signal<float>* inGradient);
        const SimFramework::Signal<float>* OutForce() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:
        // Parameters
        float mass;
        const float g=9.81;

        // Signals
        const SimFramework::Signal<float>* m_Gradient;
        SimFramework::Signal<float> m_Force;
    };














}; // namespace Models


#endif //FRAMEWORK_VEHICLECOMPONENTS_H
