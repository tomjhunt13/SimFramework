#ifndef FRAMEWORK_TRANSMISSIONSTANDALONE_H
#define FRAMEWORK_TRANSMISSIONSTANDALONE_H

#include <vector>
#include "Eigen/Dense"
#include "SimFramework/Components.h"

namespace Models {


    class LinearTrigger : public SimFramework::TriggerFunction {
    public:
        LinearTrigger() {
            this->m_Default = 0.f;
            this->t_end = 1.f;
        }

        float Evaluate(float t) {
            return 1.f * (this->t_end - t) / (this->t_end);
        }
    };

    struct GearRatios
    {
        std::vector<float> ratios = {3.f, 2.f, 1.5, 1.f};
    };


    class Transmission : public SimFramework::Model {

    public:
        Transmission() {

            float J_eff = 1.f;
            float Ratio =  2.f;

            Eigen::Matrix<float, 1, 1> A;
            A << 0.f;

            Eigen::Matrix<float, 1,2> B;
            B << 1.f / J_eff, - Ratio / J_eff;

            Eigen::Matrix<float, 2, 1> C;
            C << 1.f, Ratio;

            Eigen::Matrix<float, 2, 2> D;
            D << 0.f, 0.f, 0.f, 0.f;

            this->m_BStates.SetMatrices(A, B, C, D);
            Eigen::Vector<float, 1> initState;
            initState << 0;


            this->m_BClutchIn.Configure(&(this->m_SClutchIn), 0.f);
            this->m_BTyreIn.Configure(&(this->m_STyreIn), 0.f);
            this->m_BBlend.Configure(&(this->m_SClutchIn), &(this->m_SConst), &(this->m_STrig), &(this->m_SAugmented));
            this->m_BTrig.Configure(&(this->m_STrig));
            this->m_BConst.Configure(&(this->m_SConst), 0);


            this->m_BVec.Configure({&(this->m_SAugmented),  &(this->m_STyreIn)}, &(this->m_STorqueVec));

            this->m_BStates.Configure(&(this->m_STorqueVec), &(this->m_SSpeeds), initState);
            this->m_BMask.Configure(&(this->m_SSpeeds), {&(this->m_SClutchOut), &(this->m_STyreOut)}, {0, 1});
            this->m_BOutClutch.Configure(&(this->m_SClutchOut), 0.f);
            this->m_BOutTyre.Configure(&(this->m_STyreOut), 0.f);



            this->RegisterBlocks(
                    {&(this->m_BClutchIn), &(this->m_BTyreIn), &(this->m_BConst)},
                    {&(this->m_BStates)},
                    {&(this->m_BTrig), &(this->m_BBlend), &(this->m_BVec), &(this->m_BMask)},
                    {&(this->m_BOutTyre), &(this->m_BOutClutch)});
        };

        void ShiftUp()
        {
            // Ignore if in top gear


            // Trigger trigger  block

            // Change gear

        };
        void ShiftDown()
        {
            // Ignore if in bottom gear

            // Trigger trigger block

            // Change gear
        };


        SimFramework::Input<float>* ClutchInBlock() {
            return &(this->m_BClutchIn);
        };

        SimFramework::Input<float>* TyreInBlock() {
            return &(this->m_BTyreIn);
        };

        SimFramework::Output<float>* ClutchOutBlock() {
            return &(this->m_BOutClutch);
        };

        SimFramework::Output<float>* TyreOutBlock() {
            return &(this->m_BOutTyre);
        };

        LinearTrigger* TriggerBlock() {
            return &(this->m_BTrig);
        };


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
