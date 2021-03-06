#include "gtest/gtest.h"

#include "SimFramework/Framework.h"
#include "SimModels/Wheel.h"



TEST(Tyre, DefaultValues) {
    // Objects
    SimFramework::Signal<float> inRotationalSpeed;
    SimFramework::Signal<float>inLinearSpeed;
    Models::Tyre tyre;
    tyre.Configure(&inRotationalSpeed, &inLinearSpeed);

    // Test
    inRotationalSpeed.Write(100.f);
    inLinearSpeed.Write(10.f);
    tyre.Update();
    ASSERT_FLOAT_EQ(tyre.OutForce()->Read(), 4572.609793);
    ASSERT_FLOAT_EQ(tyre.OutTorque()->Read(), 914.521958);
}
