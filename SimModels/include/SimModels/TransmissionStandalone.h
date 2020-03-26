#ifndef FRAMEWORK_TRANSMISSIONSTANDALONE_H
#define FRAMEWORK_TRANSMISSIONSTANDALONE_H

#include <vector>
#include "Eigen/Dense"
#include "SimFramework/Components.h"
#include "SimModels/VehicleComponents.h"

namespace Models {

    struct TransmissionParameters
    {
        // Transmission
        std::vector<float> GearRatios = {0.07, 0.14, 0.23, 0.32, 0.41, 0.5};
        float TransmissionInertia = 1.f;

        // Brake
        float BrakeFrictionCoefficient = 0.9;
        float BrakeRadius = 0.15;
        float BrakeCylinderDiameter = 0.01;
        float MaxBrakePressure = 500000;
        int BrakeCylindersPerWheel = 2;
    };

    struct TransmissionBlocks {
        SimFramework::Input<float>* ClutchInBlock;
        SimFramework::Input<float>* TyreInBlock;
        SimFramework::Input<float>* BrakePressureIn;
        SimFramework::Output<float>* ClutchOutBlock;
        SimFramework::Output<float>* TyreOutBlock;
    };

    class TransmissionStandalone : public SimFramework::System {

    public:
        TransmissionStandalone();
        void SetParameters(TransmissionParameters parameters);
        void ShiftUp();
        void ShiftDown();
        TransmissionBlocks Blocks();
        int CurrentGear();

    private:
        // Signals
        SimFramework::Signal<float> m_SClutchIn;
        SimFramework::Signal<float> m_STyreIn;
        SimFramework::Signal<float> m_SBrakePressureIn;
        SimFramework::Signal<float> m_SClutchOut;
        SimFramework::Signal<float> m_STyreOut;

        // Blocks
        SimFramework::Input<float> m_BClutchIn;
        SimFramework::Input<float> m_BTyreIn;
        SimFramework::Input<float> m_BBrakePressure;
        SimFramework::Output<float> m_BOutClutch;
        SimFramework::Output<float> m_BOutTyre;

        // Subsytems
        Transmission m_Transmission;
    };

}; // namespace Models

#endif //FRAMEWORK_TRANSMISSIONSTANDALONE_H
