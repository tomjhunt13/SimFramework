#include "Vehicle/Clutch.h"

#include "SimFramework/Utilities.h"


namespace Vehicle {

    Clutch::Clutch(SimFramework::Signal<float> *innerSpeed, SimFramework::Signal<float> *outerSpeed,
                   SimFramework::Signal<float> *transmittedTorque) :
                   m_InnerSpeed(innerSpeed), m_OuterSpeed(outerSpeed), m_OutputTorque(transmittedTorque) {};

    void Clutch::Read()
    {
        this->m_InnerSpeedCopy = this->m_InnerSpeed->Read();
        this->m_OuterSpeedCopy = this->m_OuterSpeed->Read();
    };

    void Clutch::Write()
    {
        this->m_OutputTorque->Write(this->m_OutputTorqueCopy);
    };

    void Clutch::Update(float t_np1)
    {
        // Convert units of speed
        float speedInnerRPM = SimFramework::RadiansPerSecondToRPM(this->m_InnerSpeedCopy) / 1000.f;
        float speedOuterRPM = SimFramework::RadiansPerSecondToRPM(this->m_OuterSpeedCopy) / 1000.f;

        // Get relative speed
        float relativeSpeed = speedInnerRPM - speedOuterRPM;

        if (relativeSpeed >= this->m_EngagementSpeed)
        {
            this->m_OutputTorqueCopy = this->m_TorqueCapacity * (relativeSpeed * relativeSpeed - this->m_EngagementSpeed * this->m_EngagementSpeed);
        }

        else
        {
            this->m_OutputTorqueCopy = 0;
        }
    }

    void Clutch::Init(float t_0) {};

} // namespace Vehicle