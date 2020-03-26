#include "SimModels/TransmissionStandalone.h"


namespace Models {

    TransmissionStandalone::TransmissionStandalone() {

        // Configure blocks
        this->m_ClutchIn.Configure(0.f);
        this->m_TyreIn.Configure(0.f);
        this->m_BrakePressure.Configure(0.f);
        this->m_OutClutch.Configure(this->m_Transmission.OutClutchSpeed(), 0.f);
        this->m_OutTyre.Configure(this->m_Transmission.OutTyreSpeed(), 0.f);

        // Configure subsystems
        this->m_Transmission.Configure(this->m_ClutchIn.OutSignal(), this->m_TyreIn.OutSignal(), this->m_BrakePressure.OutSignal());

        SimFramework::BlockList list = {{&(this->m_ClutchIn), &(this->m_TyreIn), &(this->m_BrakePressure)},
                                        {},
                                        {},
                                        {&(this->m_OutTyre),  &(this->m_OutClutch)},
                                        {&(this->m_Transmission)}};

        this->RegisterBlocks(list);
    }

    void TransmissionStandalone::SetParameters(TransmissionParameters parameters)
    {
        this->m_Transmission.SetParameters(
                parameters.GearRatios, parameters.TransmissionInertia,
                parameters.BrakeFrictionCoefficient, parameters.BrakeRadius, parameters.BrakeCylinderDiameter, parameters.MaxBrakePressure, parameters.BrakeCylindersPerWheel);
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
        return {&(this->m_ClutchIn), &(this->m_TyreIn), &(this->m_BrakePressure), &(this->m_OutClutch), &(this->m_OutTyre)};
    };

    int TransmissionStandalone::CurrentGear()
    {
        return this->m_Transmission.CurrentGear();
    }

}; // namespace Models