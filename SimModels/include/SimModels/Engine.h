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

        SimFramework::BlockList Blocks() override;
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;

    private:
        // Blocks
        SimFramework::LookupTable2D m_EngineMap;
        SimFramework::Vectorise<float, Eigen::Vector2f> m_TorqueVector;
        SimFramework::StateSpace<Eigen::Vector2f, Eigen::Vector<float, 1>, 2, 1, 1> m_Inertia;
        SimFramework::Mask<Eigen::Matrix<float, 1, 1>, float, 1> m_SpeedMask;
    };

}; // namespace Models


#endif //FRAMEWORK_ENGINE_H
