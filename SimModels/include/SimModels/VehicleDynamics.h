#ifndef FRAMEWORK_VEHICLEDYNAMICS_H
#define FRAMEWORK_VEHICLEDYNAMICS_H

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

#include "SimModels/VehicleComponents.h"

namespace Models {


    class VehicleDynamics : public SimFramework::Subsystem
    {
    public:
        VehicleDynamics();

        void SetParameters(float initialPosition=0.f, float initialVelocity=0.f, float mass=1000.f, float Cd=0.3, float A=2.5, float rho=1.225, float rollingResistance=0.015);

        void Configure(
                const SimFramework::Signal<float>* inTyreForce,
                const SimFramework::Signal<float>* inGradient);

        const SimFramework::Signal<float>* OutVehiclePosition() const;
        const SimFramework::Signal<float>* OutVehicleVelocity() const;

        SimFramework::BlockList Blocks() override;
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;

    private:
        // Blocks
        SimFramework::ConstantBlock<float> m_ConstWeight;
        AeroDrag m_AeroDrag;
        Gravity m_Gravity;
        CoulombFriction m_RollingResistance;
        SimFramework::Vectorise<float, Eigen::Vector<float, 4>> m_Vectorise;
        SimFramework::StateSpace<Eigen::Vector<float, 4>, Eigen::Vector2f, 4, 2, 2> m_StateSpace;
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_Mask;

    };

}; // namespace Models

#endif //FRAMEWORK_VEHICLEDYNAMICS_H
