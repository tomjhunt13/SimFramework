#ifndef SIMFRAMEWORK_ENGINE_H
#define SIMFRAMEWORK_ENGINE_H

#include <string>
#include "Framework.h"
#include "Interpolation.h"

class Engine : public SimFramework::Block
{
public:
    Engine(SimFramework::Signal<float>* inSpeed, SimFramework::Signal<float>* inThrottle, SimFramework::Signal<float>* outTorque, std::string engineJSON);

    // Block API
    void Read() override;
    void Write() override;
    void Update(float t_np1) override;
    void Init(float t_0) override;

private:
    // Signals
    SimFramework::Signal<float>* m_InSpeed;
    SimFramework::Signal<float>* m_InThrottle;
    SimFramework::Signal<float>* m_OutTorque;

    // Copies
    float m_Speed;
    float m_Throttle;
    float m_Torque;

    SimFramework::Table3D m_EngineData;
};


#endif //SIMFRAMEWORK_ENGINE_H
