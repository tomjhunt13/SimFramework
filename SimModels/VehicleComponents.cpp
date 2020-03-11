#include "SimModels/VehicleComponents.h"

namespace Models {

    void Clutch::Configure(SimFramework::Signal<float>* inEngineSpeed, SimFramework::Signal<float>* outClutchTorque)
    {
        this->m_InEngineSpeed = inEngineSpeed;
        this->m_OutClutchTorque = outClutchTorque;
    };

    void Clutch::Read()
    {
        this->m_InSpeedCopy = this->m_InEngineSpeed->Read();
    };

    void Clutch::Write()
    {
        this->m_OutClutchTorque->Write(this->m_OutTorqueCopy);
    };

    void Clutch::Update(float dt)
    {
        float speed = SimFramework::RadiansPerSecondToRPM(this->m_InSpeedCopy) / 1000.f;
        this->m_OutTorqueCopy = this->m_TorqueCapacity * ( speed * speed - this->m_EngagementSpeed * this->m_EngagementSpeed);
    };

    void Clutch::Init(float t_0) {};


    LinearTrigger::LinearTrigger()
    {
        this->m_Default = 0.f;
        this->t_end = 1.f;
    }

    float LinearTrigger::Evaluate(float t) {
        return 1.f * (this->t_end - t) / (this->t_end);
    }

}; // namespace Models