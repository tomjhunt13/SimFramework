#include "SimModels/TransmissionStandalone.h"


namespace Models {

    LinearTrigger::LinearTrigger()
    {
        this->m_Default = 0.f;
        this->t_end = 1.f;
    }

    float LinearTrigger::Evaluate(float t) {
        return 1.f * (this->t_end - t) / (this->t_end);
    }


    Transmission::Transmission() {

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

    void Transmission::ShiftUp()
    {
        // Ignore if in top gear


        // Trigger trigger  block

        // Change gear

    };
    void Transmission::ShiftDown()
    {
        // Ignore if in bottom gear

        // Trigger trigger block

        // Change gear
    };


    TransmissionBlocks Transmission::Blocks() {
        return {&(this->m_BClutchIn), &(this->m_BTyreIn), &(this->m_BOutClutch), &(this->m_BOutTyre), &(this->m_BTrig)};
    };

}; // namespace Models