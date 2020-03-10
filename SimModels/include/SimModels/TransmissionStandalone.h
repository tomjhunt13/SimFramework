#ifndef FRAMEWORK_TRANSMISSIONSTANDALONE_H
#define FRAMEWORK_TRANSMISSIONSTANDALONE_H

#include <vector>
#include "Eigen/Dense"
#include "SimFramework/Components.h"

namespace Models {


    struct GearRatios
    {
        std::vector<float> ratios = {3.f, 2.f, 1.5, 1.f};
    };


    class LinearTrigger : public SimFramework::TriggerFunction {
    public:
        LinearTrigger();
        float Evaluate(float t);
    };

    struct TransmissionBlocks {
        SimFramework::Input<float>* ClutchInBlock;
        SimFramework::Input<float>* TyreInBlock;
        SimFramework::Output<float>* ClutchOutBlock;
        SimFramework::Output<float>* TyreOutBlock;
        LinearTrigger* TriggerBlock;
    };



    class Transmission : public SimFramework::Model {

    public:
        Transmission();

        void ShiftUp();
        void ShiftDown();
        TransmissionBlocks Blocks();


    private:

        // Parameters
        GearRatios m_Ratios;
        int m_GearIndex;
        float m_Inertia;

        // Signals
        SimFramework::Signal<float> m_SClutchIn;
        SimFramework::Signal<float> m_STyreIn;
        SimFramework::Signal<float> m_SConst;
        SimFramework::Signal<float> m_STrig;
        SimFramework::Signal<float> m_SAugmented;
        SimFramework::Signal<Eigen::Vector2f> m_STorqueVec;
        SimFramework::Signal<Eigen::Vector2f> m_SSpeeds;
        SimFramework::Signal<float> m_SClutchOut;
        SimFramework::Signal<float> m_STyreOut;

        // Blocks
        SimFramework::Input<float> m_BClutchIn;
        SimFramework::Input<float> m_BTyreIn;

        SimFramework::LinearBlend<float> m_BBlend;
        LinearTrigger m_BTrig;
        SimFramework::ConstantBlock<float> m_BConst;

        SimFramework::Vectorise<float, Eigen::Vector2f> m_BVec;
        SimFramework::StateSpace<Eigen::Vector2f, Eigen::Vector2f, 2, 1, 2> m_BStates;

        SimFramework::Mask<Eigen::Vector2f, float> m_BMask;

        SimFramework::Output<float> m_BOutClutch;
        SimFramework::Output<float> m_BOutTyre;
    };

}; // namespace Models

#endif //FRAMEWORK_TRANSMISSIONSTANDALONE_H
