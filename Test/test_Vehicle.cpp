#include "gtest/gtest.h"

#include "SimFramework/Framework.h"
#include "SimModels/VehicleComponents.h"



TEST(Tyre, DefaultValues) {
    // Objects
    SimFramework::Signal<float> inRotationalSpeed;
    SimFramework::Signal<float>inLinearSpeed;
    SimFramework::Signal<float> outForce;
    SimFramework::Signal<float> outTorque;
    Models::Tyre tyre;
    tyre.Configure(&inRotationalSpeed, &inLinearSpeed, &outForce, &outTorque);

    // Test
    inRotationalSpeed.Write(100.f);
    inLinearSpeed.Write(10.f);
    tyre.Update();
    ASSERT_FLOAT_EQ(outForce.Read(), 4572.609793);
    ASSERT_FLOAT_EQ(outTorque.Read(), 914.521958);
}
