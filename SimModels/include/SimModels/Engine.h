#ifndef FRAMEWORK_ENGINE_H
#define FRAMEWORK_ENGINE_H

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

namespace Models {

    class Engine : public SimFramework::Subsystem {
    public:
        Engine();

        void SetParameters(std::string engineJSON="/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json");

        void Configure(
                const SimFramework::Signal<float>* inThrottle,
                const SimFramework::Signal<float>* inSpeed);

        const SimFramework::Signal<float>* OutEngineTorque() const;
        const SimFramework::Signal<float>* OutFuelRate() const;
        const SimFramework::Signal<float>* OutFuelCumulative() const;

        SimFramework::BlockList Blocks() override;
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;

    private:
        // Blocks
        SimFramework::LookupTable2D m_TorqueMap;
        SimFramework::LookupTable2D m_FuelMap;
        SimFramework::Vectorise<float, Eigen::Vector<float, 1>> m_FuelIntegratorInput;
        SimFramework::StateSpace<Eigen::Vector<float, 1>, Eigen::Vector<float, 1>, 1, 1, 1> m_FuelIntegrator;
        SimFramework::Mask<Eigen::Vector<float, 1>, float, 1> m_FuelOutputMask;
    };

}; // namespace Models


#endif //FRAMEWORK_ENGINE_H
