#include "SimModels/Vehicle.h"


namespace Models {


    void Vehicle::ShiftUp()
    {
        // Ignore if in top gear
        if (this->m_GearIndex == this->m_Ratios.size() - 1)
        {
            return;
        }

        // Else increment gear
        this->m_GearIndex += 1;

        // Trigger trigger  block
        this->m_BTrig.Trigger();

        // Change gear
        this->SetGearRatio(this->m_GearIndex);

    };
    void Vehicle::ShiftDown()
    {
        // Ignore if in bottom gear
        if (this->m_GearIndex == 0)
        {
            return;
        }

        // Else decrement gear
        this->m_GearIndex -= 1;

        // Trigger trigger block
        this->m_BTrig.Trigger();

        // Change gear
        this->SetGearRatio(this->m_GearIndex);
    };

    int Vehicle::CurrentGear()
    {
        return this->m_GearIndex + 1;
    }

    void Vehicle::SetGearRatio(int gearIndex)
    {
        Eigen::Matrix<float, 1, 1> A;
        A << 0.f;

        Eigen::Matrix<float, 1,2> B;
        B << 1.f / this->m_EffectiveInertia, - this->m_Ratios[this->m_GearIndex] / this->m_EffectiveInertia;

        Eigen::Matrix<float, 2, 1> C;
        C << 1.f, this->m_Ratios[this->m_GearIndex];

        Eigen::Matrix<float, 2, 2> D;
        D << 0.f, 0.f, 0.f, 0.f;

        this->m_BStates.SetMatrices(A, B, C, D);
    }





}; // namespace Models