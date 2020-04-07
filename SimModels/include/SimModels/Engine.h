#ifndef FRAMEWORK_ENGINE_H
#define FRAMEWORK_ENGINE_H

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

namespace Models {

    class Engine : public SimFramework::Subsystem {
    public:
        Engine();

        void SetParameters(std::string engineJSON="/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json", float initialSpeed=200.f, float J=1.f, float b=0.05);

        void Configure(
                const SimFramework::Signal<float>* inThrottle,
                const SimFramework::Signal<float>* inLoadTorque);

        const SimFramework::Signal<float>* OutEngineSpeed() const;
        const SimFramework::Signal<float>* OutFuelRate() const;
        const SimFramework::Signal<float>* OutFuelCumulative() const;

        SimFramework::BlockList Blocks() override;
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;

    private:
        // Blocks
        SimFramework::LookupTable2D m_TorqueMap;
        SimFramework::LookupTable2D m_FuelMap;
        SimFramework::Vectorise<float, Eigen::Vector3f> m_InputVector;
        SimFramework::StateSpace<Eigen::Vector3f, Eigen::Vector2f, 3, 2, 2> m_StateSpace;
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_OutputMask;
    };

}; // namespace Models


#endif //FRAMEWORK_ENGINE_H
