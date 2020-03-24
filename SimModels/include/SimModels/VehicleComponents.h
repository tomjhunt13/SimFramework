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
                SimFramework::Signal<float>* inEngineSpeed, SimFramework::Signal<float>* inTransmissionSpeed,
                SimFramework::Signal<float>* inClutchStiffness, SimFramework::Signal<float>* outClutchTorque);

        std::vector<SimFramework::SignalBase*> InputSignals() override;
        std::vector<SimFramework::SignalBase*> OutputSignals() override;
        void Update() override;

    private:
        // Signals
        SimFramework::Signal<float>* m_InEngineSpeed;
        SimFramework::Signal<float>* m_InTransmissionSpeed;
        SimFramework::Signal<float>* m_InClutchStiffness;
        SimFramework::Signal<float>* m_OutClutchTorque;
    };


    class ClutchLowSpeedEngagement : public SimFramework::Function
    {
    public:
        ClutchLowSpeedEngagement(float threshold = 50.f, std::string name = "Clutch Low Speed Engagement");

        void Configure(
                SimFramework::Signal<float>* inTransmissionSpeed, SimFramework::Signal<float>* inThrottle,
                SimFramework::Signal<float>* outEngagement);

        std::vector<SimFramework::SignalBase*> InputSignals() override;
        std::vector<SimFramework::SignalBase*> OutputSignals() override;
        void Update() override;

    private:
        // Parameters
        float m_SpeedThreshold;
        float m_ThrottleThreshold = 0.02;
        bool m_Accelerating;

        // Signals
        SimFramework::Signal<float>* m_InTransmissionSpeed;
        SimFramework::Signal<float>* m_InThrottle;
        SimFramework::Signal<float>* m_OutEngagement;
    };


    class CentrifugalClutch : public SimFramework::Function
    {
    public:
        CentrifugalClutch(std::string = "CentrifugalClutch");

        void Configure(SimFramework::Signal<float>* inEngineSpeed, SimFramework::Signal<float>* outClutchTorque);

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
        LinearTrigger(std::string name = "Linear Trigger");
        void SetParameters(float defaultValue = 0.f, float t_end = 1.f);

        float Evaluate(float t);
    };


    class Tyre : public SimFramework::Function
    {
    public:
        Tyre(std::string name = "Tyre");

        void Configure(SimFramework::Signal<float>* inRotationalSpeed,
                       SimFramework::Signal<float>* inLinearSpeed,
                       SimFramework::Signal<float>* outForce,
                       SimFramework::Signal<float>* outTorque);

        void SetParameters(float radius=0.2, float Fz=15000,  float B=10, float C=1.9, float D=1, float E=0.97);

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
        float V_threshold = 0.001;

        // Signals
        SimFramework::Signal<float>* m_RotationalSpeed;
        SimFramework::Signal<float>* m_LinearSpeed;
        SimFramework::Signal<float>* m_Force;
        SimFramework::Signal<float>* m_Torque;
    };


    class AeroDrag : public SimFramework::Function
    {
    public:
        AeroDrag(std::string name="Aero Drag");

        void SetParameters(float Cd=0.3, float A=2.5, float rho=1.225);

        void Configure(SimFramework::Signal<float>* inSpeed, SimFramework::Signal<float>* outForce);
        std::vector<SimFramework::SignalBase*> InputSignals() override;
        std::vector<SimFramework::SignalBase*> OutputSignals() override;
        void Update() override;

    private:
        // Parameters
        float Cd;
        float A;
        float rho;

        // Signals
        SimFramework::Signal<float>* m_Speed;
        SimFramework::Signal<float>* m_Force;
    };


    class DiscBrake : public SimFramework::Function
    {
    public:
        void Configure(SimFramework::Signal<float>* inBrakePressure, SimFramework::Signal<float>* inWheelSpeed, SimFramework::Signal<float>* outBrakeTorque);
        void SetParameters(float mu=0.9, float R=0.15, float D=0.01, float maxBrakePressure=50000, int N=2);

        std::vector<SimFramework::SignalBase*> InputSignals() override;
        std::vector<SimFramework::SignalBase*> OutputSignals() override;
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
        SimFramework::Signal<float>* m_BrakePressure;
        SimFramework::Signal<float>* m_WheelSpeed;
        SimFramework::Signal<float>* m_BrakeTorque;
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
                SimFramework::Signal<float>* inThrottle,
                SimFramework::Signal<float>* inLoadTorque,
                SimFramework::Signal<float>* outEngineSpeed);

        SimFramework::BlockList Blocks() override;

    private:
        // Signals
        SimFramework::Signal<Eigen::Vector<float, 1>> m_SEngineSpeed_;
        SimFramework::Signal<float> m_SEngineTorque;
        SimFramework::Signal<Eigen::Vector2f> m_STorqueInput;

        // Blocks
        SimFramework::LookupTable2D m_BEngineMap;
        SimFramework::Vectorise<float, Eigen::Vector2f> m_BTorqueVector;
        SimFramework::StateSpace<Eigen::Vector2f, Eigen::Vector<float, 1>, 2, 1, 1> m_BInertia;
        SimFramework::Mask<Eigen::Matrix<float, 1, 1>, float> m_BMask;
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
                SimFramework::Signal<float>* inClutchTorque,
                SimFramework::Signal<float>* inTyreTorque,
                SimFramework::Signal<float>* inBrakePressure,
                SimFramework::Signal<float>* outClutchSpeed,
                SimFramework::Signal<float>* outTyreSpeed);

        SimFramework::BlockList Blocks() override;

    private:
        void SetGearRatio(int gearIndex);

        // Parameters
        std::vector<float> m_Ratios;
        int m_GearIndex;
        float m_EffectiveInertia;

        // Signals
        SimFramework::Signal<Eigen::Vector3f> m_STorqueVec;
        SimFramework::Signal<Eigen::Vector2f> m_SSpeeds;
        SimFramework::Signal<float> m_SBrakeTorque;

        // Blocks
        DiscBrake m_BDiscBrake;
        SimFramework::ConstantBlock<float> m_BConst;
        SimFramework::Vectorise<float, Eigen::Vector3f> m_BVec;
        SimFramework::StateSpace<Eigen::Vector3f, Eigen::Vector2f, 3, 1, 2> m_BStates;
        SimFramework::Mask<Eigen::Vector2f, float> m_BMask;
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
        SimFramework::Mask<Eigen::Vector2f, float> m_BMask;

    };




}; // namespace Models


#endif //FRAMEWORK_VEHICLECOMPONENTS_H
