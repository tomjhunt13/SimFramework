#ifndef FRAMEWORK_WHEEL_H
#define FRAMEWORK_WHEEL_H

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"
#include "SimModels/VehicleComponents.h"

namespace Models {

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

    class Wheel : public SimFramework::Subsystem {
    public:
        void SetParameters(float peakBreakForce=100.f, float wheelRadius=0.2);

        void Configure(
                const SimFramework::Signal<float>* inBrake,
                const SimFramework::Signal<float>* inRotationalSpeed,
                const SimFramework::Signal<float>* inLinearSpeed);

        const SimFramework::Signal<float>* OutForce() const;
        const SimFramework::Signal<float>* OutTorque() const;

        SimFramework::BlockList Blocks() override;
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;


    private:
        // Blocks
        Tyre m_Tyre;
        CoulombFriction m_Brake;
        SimFramework::SummingJunction<float> m_Sum;
    };


}; // namespace Models



#endif //FRAMEWORK_WHEEL_H
