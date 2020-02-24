#include "Engine.h"

// Private includes
#include <fstream>
#include "nlohmann/json.hpp"


Engine::Engine(SimFramework::Signal<float>* inSpeed, SimFramework::Signal<float>* inThrottle, SimFramework::Signal<float>* outTorque, std::string engineJSON)
                : m_InSpeed(inSpeed), m_InThrottle(inThrottle), m_OutTorque(outTorque)
{
    // Read file to fill m_EngineData
    std::ifstream fileObject;
    fileObject.open (engineJSON);
    nlohmann::json js = nlohmann::json::parse(fileObject);
    std::vector<float> speed = js["speed"];
    std::vector<float> throttle = js["throttle"];
    std::vector<std::vector<float>> torque = js["torque"];

    this->m_EngineData.x = speed;
    this->m_EngineData.y = throttle;
    this->m_EngineData.z = torque;

    fileObject.close();
}


void Engine::Read()
{
    this->m_Speed = this->m_InSpeed->Read();
    this->m_Throttle = this->m_InThrottle->Read();
};

void Engine::Write()
{
    this->m_OutTorque->Write(this->m_Torque);
};

void Engine::Update(float t_np1)
{
    this->m_Torque = SimFramework::InterpTable2D(this->m_EngineData, this->m_Speed, this->m_Throttle);
};

void Engine::Init(float t_0) {}

