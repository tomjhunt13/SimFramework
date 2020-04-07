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

        void SetParameters(float initialPosition=0.f, float initialVelocity=0.f, float mass=1000.f, float Cd=0.3, float A=2.5, float rho=1.225, float peakBrakeForce=300.f);

        void Configure(
                const SimFramework::Signal<float>* inTyreForce,
                const SimFramework::Signal<float>* inGradient,
                const SimFramework::Signal<float>* inBrakePedal);

        const SimFramework::Signal<float>* OutVehiclePosition() const;
        const SimFramework::Signal<float>* OutVehicleVelocity() const;

        SimFramework::BlockList Blocks() override;
        std::vector<std::pair<std::string, const SimFramework::SignalBase *> > LogSignals() override;

    private:
        // Blocks
        AeroDrag m_AeroDrag;
        Gravity m_Gravity;
        CoulombFriction m_Brake;
        SimFramework::Vectorise<float, Eigen::Vector4f> m_Vectorise;
        SimFramework::StateSpace<Eigen::Vector4f, Eigen::Vector2f, 4, 2, 2> m_StateSpace;
        SimFramework::Mask<Eigen::Vector2f, float, 2> m_Mask;

    };

}; // namespace Models

#endif //FRAMEWORK_VEHICLEDYNAMICS_H
