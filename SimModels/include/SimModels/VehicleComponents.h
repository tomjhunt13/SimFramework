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
        Clutch(std::string = "Clutch");

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
        LinearTrigger(float defaultValue = 0.f, float t_end = 1.f, std::string name = "Linear Trigger");
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


    class AeroDrag : public SimFramework::Function
    {
    public:
        AeroDrag(std::string name="Aero Drag", float Cd=0.3, float A=2.5, float rho=1.225);

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
        // TODO: currently brake pressure is a value between 0 and 1. Should consider putting gain outside
    public:
        void Configure(SimFramework::Signal<float>* inBrakePressure, SimFramework::Signal<float>* inWheelSpeed, SimFramework::Signal<float>* outBrakeTorque);
        void SetParameters(float mu=0.9, float R=0.15, float D=0.01, int N=2);

        std::vector<SimFramework::SignalBase*> InputSignals() override;
        std::vector<SimFramework::SignalBase*> OutputSignals() override;
        void Update() override;

    private:
        // Parameters
        float mu;   // Friction coefficient
        float R;    // Average radius of pad
        float D;    // Diameter of cylinder
        int N;      // Number of cylinders
        float m_BrakeConstant;

        // Signals
        SimFramework::Signal<float>* m_BrakePressure;
        SimFramework::Signal<float>* m_WheelSpeed;
        SimFramework::Signal<float>* m_BrakeTorque;
    };


    class Engine : public SimFramework::Subsystem {
    public:
        Engine(std::string engineJSON="/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json", float initialSpeed=200.f, float J=1.f, float b=0.05);

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
        void ShiftUp();
        void ShiftDown();
        int CurrentGear();

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
        std::vector<float> m_Ratios = {0.5, 1.f, 1.5, 2.f, 3.f};
        int m_GearIndex;
        float m_EffectiveInertia = 1.f;

        // Signals
        SimFramework::Signal<float> m_SConst;
        SimFramework::Signal<float> m_STrig;
        SimFramework::Signal<float> m_SAugmented;
        SimFramework::Signal<Eigen::Vector3f> m_STorqueVec;
        SimFramework::Signal<Eigen::Vector2f> m_SSpeeds;
        SimFramework::Signal<float> m_SBrakeTorque;

        // Blocks
        DiscBrake m_BDiscBrake;
        SimFramework::LinearBlend<float> m_BBlend;
        LinearTrigger m_BTrig;
        SimFramework::ConstantBlock<float> m_BConst;
        SimFramework::Vectorise<float, Eigen::Vector3f> m_BVec;
        SimFramework::StateSpace<Eigen::Vector3f, Eigen::Vector2f, 3, 1, 2> m_BStates;
        SimFramework::Mask<Eigen::Vector2f, float> m_BMask;
    };


    class VehicleDynamics : public SimFramework::Subsystem
    {
    public:
        VehicleDynamics(float initialPosition=0.f, float initialVelocity=0.f, float mass=1000.f, float Cd=0.3, float A=2.5, float rho=1.225);

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
