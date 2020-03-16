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
        void Configure(SimFramework::Signal<float>* inEngineSpeed, SimFramework::Signal<float>* inClutchSpeed, SimFramework::Signal<float>* outClutchTorque);

        std::vector<SimFramework::SignalBase*> InputSignals() override;
        std::vector<SimFramework::SignalBase*> OutputSignals() override;
        void Update() override;

    private:
        // Parameters
        float m_EngagementSpeed = 1.5f; // [1000 rev/min]
        float m_TorqueCapacity = 30.f; // [Nm @ engagement speed]

        // Signals
        SimFramework::Signal<float>* m_InEngineSpeed;
        SimFramework::Signal<float>* m_OutClutchTorque;

        // Copies
        float m_InSpeedCopy;
        float m_OutTorqueCopy;
    };


    class LinearTrigger : public SimFramework::TriggerFunction {
    public:
        LinearTrigger();
        float Evaluate(float t);
    };


    class Tyre : public SimFramework::Function
    {
    public:
        void Configure(SimFramework::Signal<float>* inRotationalSpeed,
                       SimFramework::Signal<float>* inLinearSpeed,
                       SimFramework::Signal<float>* outForce,
                       SimFramework::Signal<float>* outTorque);

        void SetParameters(float radius=0.2, float Fz=5000,  float B=10, float C=1.9, float D=1, float E=0.97);

        std::vector<SimFramework::SignalBase*> InputSignals() override;
        std::vector<SimFramework::SignalBase*> OutputSignals() override;
        void Update() override;

    private:
        // Parameters
        float radius;
        float Fz;
        float B;
        float C;
        float D;
        float E;

        // Signals
        SimFramework::Signal<float>* m_RotationalSpeed;
        SimFramework::Signal<float>* m_LinearSpeed;
        SimFramework::Signal<float>* m_Force;
        SimFramework::Signal<float>* m_Torque;
    };


    class Engine : public SimFramework::Subsystem {
    public:

        void SetEngineParameters(std::string engineJSON="/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json", float J=1.f, float b=0.05);

        void Configure(
                SimFramework::Signal<float>* inThrottle,
                SimFramework::Signal<float>* inLoadTorque,
                SimFramework::Signal<float>* outEngineSpeed);

        SimFramework::BlockList Blocks() override;

    private:
        // Signals
        SimFramework::Signal<Eigen::Matrix<float, 1, 1>> m_SEngineSpeed_;
        SimFramework::Signal<float> m_SEngineTorque;
        SimFramework::Signal<float> m_SResultantTorque;

        // Blocks
        SimFramework::LookupTable2D m_BEngineMap;
        SimFramework::SummingJunction<float> m_BSum;
        SimFramework::StateSpace<float, Eigen::Matrix<float, 1, 1>, 1, 1, 1> m_BInertia;
        SimFramework::Mask<Eigen::Matrix<float, 1, 1>, float> m_BMask;
    };


    class Transmission : public SimFramework::Subsystem {

    public:
        Transmission();
        void ShiftUp();
        void ShiftDown();
        int CurrentGear();

        void Configure(
                SimFramework::Signal<float>* inClutchTorque,
                SimFramework::Signal<float>* inTyreTorque,
                SimFramework::Signal<float>* outClutchSpeed,
                SimFramework::Signal<float>* outTyreSpeed);

        SimFramework::BlockList Blocks() override;

    private:
        void SetGearRatio(int gearIndex);

        // Parameters
        std::vector<float> m_Ratios = {0.5, 1.f, 1.5, 2.f, 3.f};
        int m_GearIndex;
        float m_EffectiveInertia = 1.f;

        // Signals
        SimFramework::Signal<float> m_SConst;
        SimFramework::Signal<float> m_STrig;
        SimFramework::Signal<float> m_SAugmented;
        SimFramework::Signal<Eigen::Vector2f> m_STorqueVec;
        SimFramework::Signal<Eigen::Vector2f> m_SSpeeds;

        // Blocks
        SimFramework::LinearBlend<float> m_BBlend;
        LinearTrigger m_BTrig;
        SimFramework::ConstantBlock<float> m_BConst;
        SimFramework::Vectorise<float, Eigen::Vector2f> m_BVec;
        SimFramework::StateSpace<Eigen::Vector2f, Eigen::Vector2f, 2, 1, 2> m_BStates;
        SimFramework::Mask<Eigen::Vector2f, float> m_BMask;
    };




}; // namespace Models


#endif //FRAMEWORK_VEHICLECOMPONENTS_H
