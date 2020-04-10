#include "SimModels/LockupClutch.h"

namespace Models {

    TransmittedTorque::TransmittedTorque(std::string name) : Function(name) {};

    void TransmittedTorque::SetParameters(EngineTransmissionParameters parameters)
    {
        this->m_Parameters = parameters;
    };

    void TransmittedTorque::Configure(
            const SimFramework::Signal<float>* inSpeed,
            const SimFramework::Signal<float>* inEngineTorque,
            const SimFramework::Signal<float>* inTyreTorque)
    {
        this->m_InSpeed = inSpeed;
        this->m_InEngineTorque = inEngineTorque;
        this->m_InTyreTorque = inTyreTorque;
    };

    const SimFramework::Signal<float>* TransmittedTorque::OutTorque() const
    {
        return &(this->m_OutTorque);
    };

    std::vector<const SimFramework::SignalBase*> TransmittedTorque::InputSignals() const
    {
        return {this->m_InTyreTorque, this->m_InEngineTorque, this->m_InSpeed};
    };

    std::vector<const SimFramework::SignalBase*> TransmittedTorque::OutputSignals() const
    {
        return {&(this->m_OutTorque)};
    };

    void TransmittedTorque::Update()
    {
        // Read inputs
        float speed = this->m_InSpeed->Read();
        float engineTorque = this->m_InEngineTorque->Read();
        float tyreTorque = this->m_InTyreTorque->Read();

        // Calculate transmitted torque
        float outTorque = this->Evaluate(speed, engineTorque, tyreTorque, this->m_Parameters.G, this->m_Parameters.b_e, this->m_Parameters.b_t, this->m_Parameters.I_e, this->m_Parameters.I_t);

        // Write to output signal
        this->m_OutTorque.Write(outTorque);
    };

    float TransmittedTorque::Evaluate(float w, float T_e, float T_t, float G, float b_e, float b_t, float I_e, float I_t)
    {
        return ((I_e * b_t - I_t * b_e) * w + I_t * T_e + I_e * T_t) / (I_t + G * I_e);
    };


    void LockupClutch::SetGearRatio(float G)
    {

        // Set up state space matrices for locked state
        float inertia = G / (G * this->I_e + this->I_t);

        Eigen::Matrix<float, 1, 1> lockedA;
        lockedA << - (this->b_e + this->b_t) * inertia;

        Eigen::Matrix<float, 1, 2> lockedB;
        lockedB << inertia, - inertia;

        Eigen::Matrix<float, 2, 1> lockedC;
        lockedC << 1.f, G;

        Eigen::Matrix<float, 2, 2> lockedD = Eigen::Matrix<float, 2, 2>::Zero();

        this->LockedState.SetMatrices(lockedA, lockedB, lockedC, lockedD);

        // Set up state space matrices for unlocked state
        Eigen::Matrix<float, 2, 2> unlockedA;
        unlockedA << - this->b_e / this->I_e, 0.f, 0.f, - this->b_t / this->I_t;

        Eigen::Matrix<float, 2, 3> unlockedB;
        unlockedB << 1.f / this->I_e, - 1.f / this->I_e, 0.f, 0.f, G / this->I_t, - 1 / this->I_t;

        Eigen::Matrix<float, 2, 2> unlockedC;
        unlockedC << 1.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<float, 2, 3> unlockedD = Eigen::Matrix<float, 2, 3>::Zero();

        this->UnLockedState.SetMatrices(unlockedA, unlockedB, unlockedC, unlockedD);

    };

}; // namespace Models