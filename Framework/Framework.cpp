#include "SimFramework/Framework.h"

#include "SimFramework/Utilities.h"

namespace SimFramework {


    void Model::RegisterBlocks(
            std::vector<Block*> sources, std::vector<Block*> dynamicSystems,
            std::vector<Block*> functions, std::vector<Block*> sinks)
    {
        this->m_Sources = sources;
        this->m_DynamicSystems = dynamicSystems;
        this->m_Functions = functions;
        this->m_Sinks = sinks;
    };

    void Model::Initialise(float t_0)
    {
        for (auto i: this->m_Sources)
        {
            i->Init(t_0);
        }

        for (auto i: this->m_DynamicSystems)
        {
            i->Init(t_0);
        }

        this->UpdateFunctions(t_0);
    }

    void Model::UpdateFunctions(float dt)
    {
        for (auto i: this->m_Functions)
        {
            i->Read();
            i->Update(dt);
            i->Write();
        }
    }

    void Model::Update(float t_np1)
    {

        // Get steps
        std::vector<float> timeSteps = TimeSteps(this->m_t_n, t_np1, this->m_dtMax);

        for (float dt: timeSteps) {

            // Read
            for (auto i: this->m_DynamicSystems) {
                i->Read();
            }

            for (auto i: this->m_Sinks) {
                i->Read();
            }

            // Update everything else
            for (auto i: this->m_Sources) {
                i->Update(dt);
            }

            for (auto i: this->m_DynamicSystems) {
                i->Update(dt);
            }

            for (auto i: this->m_Sinks) {
                i->Update(dt);
            }

            // Write everything else
            for (auto i: this->m_Sources) {
                i->Write();
            }

            for (auto i: this->m_DynamicSystems) {
                i->Write();
            }

            this->UpdateFunctions(dt);
        }

        this->m_t_n = t_np1;
    };



} // namespace Framework