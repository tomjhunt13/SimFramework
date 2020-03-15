#include "SimModels/TransmissionStandalone.h"


namespace Models {

    TransmissionStandalone::TransmissionStandalone() {

       // Configure blocks
        this->m_BClutchIn.Configure(&(this->m_SClutchIn), 0.f);
        this->m_BTyreIn.Configure(&(this->m_STyreIn), 0.f);
        this->m_BOutClutch.Configure(&(this->m_SClutchOut), 0.f);
        this->m_BOutTyre.Configure(&(this->m_STyreOut), 0.f);

        // Configure subsystems
        this->m_Transmission.Configure(&(this->m_SClutchIn), &(this->m_STyreIn), &(this->m_SClutchOut), &(this->m_STyreOut));

        this->RegisterBlocks(
                {&(this->m_BClutchIn), &(this->m_BTyreIn)},
                {},
                {},
                {&(this->m_BOutTyre), &(this->m_BOutClutch)},
                {&(this->m_Transmission)});
    };

    void TransmissionStandalone::ShiftUp()
    {
        this->m_Transmission.ShiftUp();
    };

    void TransmissionStandalone::ShiftDown()
    {
        this->m_Transmission.ShiftDown();
    };

    TransmissionBlocks TransmissionStandalone::Blocks() {
        return {&(this->m_BClutchIn), &(this->m_BTyreIn), &(this->m_BOutClutch), &(this->m_BOutTyre)};
    };

    int TransmissionStandalone::CurrentGear()
    {
        return this->m_Transmission.CurrentGear();
    }

}; // namespace Models