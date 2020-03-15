#ifndef FRAMEWORK_TRANSMISSIONSTANDALONE_H
#define FRAMEWORK_TRANSMISSIONSTANDALONE_H

#include <vector>
#include "Eigen/Dense"
#include "SimFramework/Components.h"
#include "SimModels/VehicleComponents.h"

namespace Models {

    struct TransmissionBlocks {
        SimFramework::Input<float>* ClutchInBlock;
        SimFramework::Input<float>* TyreInBlock;
        SimFramework::Output<float>* ClutchOutBlock;
        SimFramework::Output<float>* TyreOutBlock;
    };

    class TransmissionStandalone : public SimFramework::Model {

    public:
        TransmissionStandalone();
        void ShiftUp();
        void ShiftDown();
        TransmissionBlocks Blocks();
        int CurrentGear();


    private:
        // Signals
        SimFramework::Signal<float> m_SClutchIn;
        SimFramework::Signal<float> m_STyreIn;
        SimFramework::Signal<float> m_SClutchOut;
        SimFramework::Signal<float> m_STyreOut;

        // Blocks
        SimFramework::Input<float> m_BClutchIn;
        SimFramework::Input<float> m_BTyreIn;
        SimFramework::Output<float> m_BOutClutch;
        SimFramework::Output<float> m_BOutTyre;

        // Subsytems
        Transmission m_Transmission;
    };

}; // namespace Models

#endif //FRAMEWORK_TRANSMISSIONSTANDALONE_H
