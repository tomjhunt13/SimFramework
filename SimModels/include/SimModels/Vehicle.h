#ifndef FRAMEWORK_VEHICLE_H
#define FRAMEWORK_VEHICLE_H

#include "SimFramework/Utilities.h"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

#include "SimModels/VehicleComponents.h"

namespace Models {

//
//
//    struct VehicleBlocks
//    {
//        SimFramework::Input<float> throttle;
//
//    };
//
//    class Vehicle : public SimFramework::Model
//    {
//
//
//        Vehicle()
//        {
//
//            // ------------- Engine -------------
//            // Initial engine speed
//            Eigen::Matrix<float, 1, 1> init;
//            init << 300.f;
//
//            // Configure blocks
//            this->m_BThrottle.Configure(&(this->m_SThrottle), 0.f);
//            this->m_BEngineMap.Configure(&(this->m_SEngineSpeed), &(this->m_SThrottle), &(this->m_SEngineTorque));
//            this->m_BSumEng.Configure({&(this->m_SEngineTorque), &(this->m_SLoadTorque)}, &(this->m_SResultantTorque), {1.f, -1.f});
//            this->m_BInertiaEng.Configure(&(this->m_SResultantTorque), &(this->m_SEngineSpeed_), init);
//            this->m_BMaskEng.Configure(&(this->m_SEngineSpeed_), {&(this->m_SEngineSpeed)}, {0});
//
//            // ------------- Clutch -------------
//            // Configure blocks
//            this->m_BClutch.Configure(&(this->m_SEngineSpeed), &(this->m_SClutchOut));
//
//        }
//
//
//        // ------------- Transmission -------------
//        void ShiftUp();
//        void ShiftDown();
//        int CurrentGear();
//
//    private:
//
//        // ------------- Engine -------------
//        // Signals
//        SimFramework::Signal<float> m_SThrottle;
//        SimFramework::Signal<float> m_SLoadTorque;
//        SimFramework::Signal<float> m_SEngineSpeed;
//        SimFramework::Signal<Eigen::Matrix<float, 1, 1>> m_SEngineSpeed_;
//        SimFramework::Signal<float> m_SEngineTorque;
//        SimFramework::Signal<float> m_SResultantTorque;
//
//        // Blocks
//        SimFramework::Input<float> m_BThrottle;
//        SimFramework::LookupTable2D m_BEngineMap;
//        SimFramework::SummingJunction<float> m_BSumEng;
//        SimFramework::StateSpace<float, Eigen::Matrix<float, 1, 1>, 1, 1, 1> m_BInertiaEng;
//        SimFramework::Mask<Eigen::Matrix<float, 1, 1>, float> m_BMaskEng;
//
//        // ------------- Clutch -------------
//        SimFramework::Signal<float> m_SClutchOut;
//        Clutch m_BClutch;
//
//
//
//        // ------------- Transmission -------------
//        // Methods
//        void SetGearRatio(int gearIndex);
//        // Parameters
//        std::vector<float> m_Ratios = {0.5, 1.f, 1.5, 2.f, 3.f};
//        int m_GearIndex;
//        float m_EffectiveInertia = 1.f;
//
//        // Signals
//        SimFramework::Signal<float> m_SConst;
//        SimFramework::Signal<float> m_STrig;
//        SimFramework::Signal<float> m_SAugmented;
//        SimFramework::Signal<Eigen::Vector2f> m_STorqueVec;
//        SimFramework::Signal<Eigen::Vector2f> m_SSpeeds;
//        SimFramework::Signal<float> m_SClutchSpeedOut;
//        SimFramework::Signal<float> m_STyreSpeedOut;
//
//        // Blocks
//        SimFramework::Input<float> m_BClutchIn;
//        SimFramework::Input<float> m_BTyreIn;
//        SimFramework::LinearBlend<float> m_BBlend;
//        LinearTrigger m_BTrig;
//        SimFramework::ConstantBlock<float> m_BConst;
//        SimFramework::Vectorise<float, Eigen::Vector2f> m_BVec;
//        SimFramework::StateSpace<Eigen::Vector2f, Eigen::Vector2f, 2, 1, 2> m_BStates;
//        SimFramework::Mask<Eigen::Vector2f, float> m_BMask;
//        SimFramework::Output<float> m_BOutClutch;
//        SimFramework::Output<float> m_BOutTyre;
//
//
//        // ------------- Tyre -------------
//
//
//        // ------------- VehicleDynamics -------------
//
//
//    };

}; // namespace Models

#endif //FRAMEWORK_VEHICLE_H
