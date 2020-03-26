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


    class ClutchLowSpeedEngagement : public SimFramework::Function
    {
    public:
        ClutchLowSpeedEngagement(float threshold = 50.f, std::string name = "Clutch Low Speed Engagement");

        void Configure(const SimFramework::Signal<float>* inTransmissionSpeed, const SimFramework::Signal<float>* inThrottle);

        const SimFramework::Signal<float>* OutEngagement() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:
        // Parameters
        float m_SpeedThreshold;
        float m_ThrottleThreshold = 0.02;
        bool m_Accelerating;

        // Signals
        const SimFramework::Signal<float>* m_InTransmissionSpeed;
        const SimFramework::Signal<float>* m_InThrottle;
        SimFramework::Signal<float> m_OutEngagement;
    };

// TODO: Remove
//
//    class CentrifugalClutch : public SimFramework::Function
//    {
//    public:
//        CentrifugalClutch(std::string = "CentrifugalClutch");
//
//        void Configure(SimFramework::Signal<float>* inEngineSpeed);
//
//        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
//        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
//        void Update() override;
//
//    private:
//        // Parameters
//        float m_EngagementSpeed = 1.5f; // [1000 rev/min]
//        float m_TorqueCapacity = 30.f; // [Nm @ engagement speed]
//
//        // Signals
//        SimFramework::Signal<float>* m_InEngineSpeed;
//        SimFramework::Signal<float>* m_OutClutchTorque;
//
//        // Copies
//        float m_InSpeedCopy;
//        float m_OutTorqueCopy;
//    };


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

        void SetParameters(float radius=0.2, float Fz=15000,  float B=10, float C=1.9, float D=1, float E=0.97);

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


    class DiscBrake : public SimFramework::Function
    {
    public:
        void Configure(const SimFramework::Signal<float>* inBrakePressure, const SimFramework::Signal<float>* inWheelSpeed);
        void SetParameters(float mu=0.9, float R=0.15, float D=0.01, float maxBrakePressure=50000, int N=2);

        const SimFramework::Signal<float>* OutTorque() const;

        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:
        // Parameters
        float mu;   // Friction coefficient
        float R;    // Average radius of pad
        float D;    // Diameter of cylinder
        int N;      // Number of cylinders
        float maxBrakePressure;
        float m_BrakeConstant;

        // Signals
        const SimFramework::Signal<float>* m_BrakePressure;
        const SimFramework::Signal<float>* m_WheelSpeed;
        SimFramework::Signal<float> m_BrakeTorque;
    };





    class VehicleController : public SimFramework::Subsystem
    {
    public:
        void SetParameters(float clutchLagTime=1.f, float clutchStiffness=1000.f);

        void Configure(SimFramework::Signal<float>* inDemandThrottle, SimFramework::Signal<float>* inTransmissionSpeed,
                       SimFramework::Signal<float>* outThrottleAugmented, SimFramework::Signal<float>* outClutchStiffness);

        void Trigger();

        SimFramework::BlockList Blocks() override;

    private:
        // Parameters
        float m_ClutchStiffness;

        // Signals
        SimFramework::Signal<float> m_SConstClutchStiffness;
        SimFramework::Signal<float> m_SConstZero;
        SimFramework::Signal<float> m_SEngagementSignal;
        SimFramework::Signal<float> m_SLowSpeedEngangement;
        SimFramework::Signal<float> m_STriggerSignal;

        // Blocks - Blending functions
        SimFramework::LinearBlend<float> m_BBlendClutchLowSpeed;
        SimFramework::LinearBlend<float> m_BBlendThrottle;
        SimFramework::LinearBlend<float> m_BBlendClutchGearShift;

        // Blocks - Blend parameters
        LinearTrigger m_BGearChangeTrigger;
        ClutchLowSpeedEngagement m_BLowSpeedEngagement;

        // Blocks - Constants
        SimFramework::ConstantBlock<float> m_BConstZero;
        SimFramework::ConstantBlock<float> m_BClutchStiffnessMax;


    };


    class Engine : public SimFramework::Subsystem {
    public:
        Engine();

        void SetParameters(std::string engineJSON="/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json", float initialSpeed=200.f, float J=1.f, float b=0.05);

        void Configure(
                const SimFramework::Signal<float>* inThrottle,
                const SimFramework::Signal<float>* inLoadTorque);

        const SimFramework::Signal<float>* OutEngineSpeed() const;

        SimFramework::BlockList Blocks() override;

    private:
        // Blocks
        SimFramework::LookupTable2D m_EngineMap;
        SimFramework::Vectorise<float, Eigen::Vector2f> m_TorqueVector;
        SimFramework::StateSpace<Eigen::Vector2f, Eigen::Vector<float, 1>, 2, 1, 1> m_Inertia;
        SimFramework::Mask<Eigen::Matrix<float, 1, 1>, float, 1> m_SpeedMask;
    };


    class Transmission : public SimFramework::Subsystem {
    public:
        void SetParameters(
                std::vector<float> gearRatios = {0.07, 0.14, 0.23, 0.32, 0.41, 0.5}, float effectiveInertia = 1.f,
                float Mu=0.9, float R=0.15, float D=0.01, float maxBrakePressure=50000, int N=2);

        bool ShiftUp();
        bool ShiftDown();
        int CurrentGear() const;

        void Configure(
                const SimFramework::Signal<float>* inClutchTorque,
                const SimFramework::Signal<float>* inTyreTorque,
                const SimFramework::Signal<float>* inBrakePressure);

        const SimFramework::Signal<float>* OutClutchSpeed() const;
        const SimFramework::Signal<float>* OutTyreSpeed() const;

        SimFramework::BlockList Blocks() override;

    private:
        void SetGearRatio();

        // Parameters
        std::vector<float> m_Ratios;
        int m_GearIndex;
        float m_EffectiveInertia;


        // Blocks
        DiscBrake m_DiscBrake;
        SimFramework::Vectorise<float, Eigen::Vector3f> m_TorqueVector;
        SimFramework::StateSpace<Eigen::Vector3f, Eigen::Vector2f, 3, 1, 2> m_States;
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_StateMask;
    };


    class VehicleDynamics : public SimFramework::Subsystem
    {
    public:
        VehicleDynamics();

        void SetParameters(float initialPosition=0.f, float initialVelocity=0.f, float mass=1000.f, float Cd=0.3, float A=2.5, float rho=1.225);

        void Configure(
                SimFramework::Signal<float>* inTyreForce,
                SimFramework::Signal<float>* outVehiclePosition,
                SimFramework::Signal<float>* outVehicleVelocity);

        SimFramework::BlockList Blocks() override;

    private:
        // Signals
        SimFramework::Signal<float> m_SAeroDrag;
        SimFramework::Signal<float> m_SGravity;
        SimFramework::Signal<Eigen::Vector3f> m_SForceVec;
        SimFramework::Signal<Eigen::Vector2f> m_SStatesVec;

        // Blocks
        AeroDrag m_BAeroDrag;
        SimFramework::Vectorise<float, Eigen::Vector3f> m_BVectorise;
        SimFramework::StateSpace<Eigen::Vector3f, Eigen::Vector2f, 3, 2, 2> m_BStateSpace;
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_BMask;

    };




}; // namespace Models


#endif //FRAMEWORK_VEHICLECOMPONENTS_H
