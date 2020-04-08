#ifndef FRAMEWORK_VEHICLECOMPONENTS_H
#define FRAMEWORK_VEHICLECOMPONENTS_H

#include <cmath>
#include "SimFramework/Utilities.h"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

namespace Models {

    class Clutch : public SimFramework::Function
    {
    public:
        Clutch(std::string name = "Clutch");

        void Configure(
                const SimFramework::Signal<float>* inEngineSpeed,
                const SimFramework::Signal<float>* inTransmissionSpeed,
                const SimFramework::Signal<float>* inClutchStiffness);

        const SimFramework::Signal<float>* OutClutchTorque() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:
        // Signals
        const SimFramework::Signal<float>* m_InEngineSpeed;
        const SimFramework::Signal<float>* m_InTransmissionSpeed;
        const SimFramework::Signal<float>* m_InClutchStiffness;
        SimFramework::Signal<float> m_OutClutchTorque;
    };





    class LinearTrigger : public SimFramework::TriggerFunction {
    public:
        LinearTrigger(std::string name = "Linear Trigger");
        void SetParameters(float defaultValue = 0.f, float t_end = 1.f);

        float Evaluate(float t);
    };


    class Tyre : public SimFramework::Function
    {
    public:
        Tyre(std::string name = "Tyre");

        void Configure(const SimFramework::Signal<float>* inRotationalSpeed,
                       const SimFramework::Signal<float>* inLinearSpeed);

        const SimFramework::Signal<float>*  OutForce() const;
        const SimFramework::Signal<float>*  OutTorque() const;

        void SetParameters(float radius=0.2, float Fz=15000, float D=1.f, float B=10, float C=1.2,  float E=0.97);

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
        float V_threshold = 0.1;

        // Signals
        const SimFramework::Signal<float>* m_RotationalSpeed;
        const SimFramework::Signal<float>* m_LinearSpeed;
        SimFramework::Signal<float> m_Force;
        SimFramework::Signal<float> m_Torque;
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
