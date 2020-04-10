#ifndef FRAMEWORK_LOCKUPCLUTCH_H
#define FRAMEWORK_LOCKUPCLUTCH_H

#include "Eigen/Dense"

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

namespace Models {

    struct EngineTransmissionParameters {
        float G;
        float b_e;
        float b_t;
        float I_e;
        float I_t;
    };


    class TransmittedTorque : public SimFramework::Function
    {
    public:
        TransmittedTorque(std::string name="Transmitted Torque");

        void SetParameters(EngineTransmissionParameters parameters);
        void Configure(
                const SimFramework::Signal<float>* inSpeed,
                const SimFramework::Signal<float>* inEngineTorque,
                const SimFramework::Signal<float>* inTyreTorque);
        const SimFramework::Signal<float>* OutTorque() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:
        float Evaluate(float w, float T_e, float T_t, float G, float b_e, float b_t, float I_e, float I_t);

        // Parameters
        EngineTransmissionParameters m_Parameters;

        // Signals
        const SimFramework::Signal<float>* m_InSpeed;
        const SimFramework::Signal<float>* m_InEngineTorque;
        const SimFramework::Signal<float>* m_InTyreTorque;
        SimFramework::Signal<float> m_OutTorque;
    };

    class LockupClutch {
    public:


        void SetGearRatio(float G);


    private:

        // System Parameters
        float b_e = 0.2;
        float b_t = 0.5;
        float I_e = 0.25;
        float I_t = 0.4;

        //
        SimFramework::LookupTable2D m_TorqueMap;
        SimFramework::LookupTable2D m_FuelMap;

        // State space
        SimFramework::StateSpace<Eigen::Vector3f, Eigen::Vector2f, 3, 2, 2> UnLockedState;
        SimFramework::StateSpace<Eigen::Vector2f, Eigen::Vector2f, 2, 1, 2> LockedState;


    };

}; // namespace Models


#endif //FRAMEWORK_LOCKUPCLUTCH_H
