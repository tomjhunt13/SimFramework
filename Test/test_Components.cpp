#include <vector>
#include "gtest/gtest.h"
#include "Eigen/Dense"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

TEST(SummingJunction, testFloat) {
    // Input Signals
    SimFramework::Signal<float> in1;
    SimFramework::Signal<float> in2;
    SimFramework::Signal<float> in3;

    // Output Signal
    SimFramework::Signal<float> out;

    // Construct block
    SimFramework::SummingJunction<float> sum;

    // Configure as [+++]
    sum.Configure({&in1, &in2, &in3}, &out, {1.f, 1.f, 1.f});

    // Write input values
    in1.Write(1.5);
    in2.Write(3.5);
    in3.Write(4.5);

    // Test
    sum.Update();
    ASSERT_FLOAT_EQ(out.Read(), 9.5);

    // Configure as [+--]
    sum.Configure({&in1, &in2, &in3}, &out, {1.f, -1.f, -1.f});

    // Test
    sum.Update();
    ASSERT_FLOAT_EQ(out.Read(), -6.5);
}


TEST(Vectorise, Vec2) {
    // Input Signals
    SimFramework::Signal<float> in1;
    SimFramework::Signal<float> in2;

    // Output Signal
    SimFramework::Signal<Eigen::Vector2f> out;

    // Construct block
    SimFramework::Vectorise<float, Eigen::Vector2f> vec;

    // Configure as [+++]
    vec.Configure({&in1, &in2}, &out);

    // Write input values
    in1.Write(1.5);
    in2.Write(3.5);

    // Test
    Eigen::Vector2f expected = {1.5, 3.5};
    vec.Update();
    ASSERT_TRUE(out.Read() == expected);
}

TEST(Vectorise, Vec3) {
    // Input Signals
    SimFramework::Signal<float> in1;
    SimFramework::Signal<float> in2;
    SimFramework::Signal<float> in3;

    // Output Signal
    SimFramework::Signal<Eigen::Vector3f> out;

    // Construct block
    SimFramework::Vectorise<float, Eigen::Vector3f> vec;

    // Configure as [+++]
    vec.Configure({&in1, &in2, &in3}, &out);

    // Write input values
    in1.Write(1.5);
    in2.Write(3.5);
    in3.Write(5.5);

    // Test
    Eigen::Vector3f expected = {1.5, 3.5, 5.5};
    vec.Update();
    ASSERT_TRUE(out.Read() == expected);
}


TEST(Mask, Vec3_1Out_1) {
    // Input Signal
    SimFramework::Signal<Eigen::Vector3f> in;

    // Output Signals
    SimFramework::Signal<float> out1;
    SimFramework::Signal<float> out2;
    SimFramework::Signal<float> out3;

    // Construct block
    SimFramework::Mask<Eigen::Vector3f, float> vec;

    // Write input values
    in.Write({1.5, 3.5, 5.5});
    out1.Write(0.f);
    out2.Write(0.f);
    out3.Write(0.f);

    // Only output to out1
    vec.Configure(&in, {&out1}, {0});
    vec.Update();
    ASSERT_FLOAT_EQ(out1.Read(), 1.5);
    ASSERT_FLOAT_EQ(out2.Read(), 0.f);
    ASSERT_FLOAT_EQ(out3.Read(), 0.f);
}

TEST(Mask, Vec3_1Out_2) {
    // Input Signal
    SimFramework::Signal<Eigen::Vector3f> in;

    // Output Signals
    SimFramework::Signal<float> out1;
    SimFramework::Signal<float> out2;
    SimFramework::Signal<float> out3;

    // Construct block
    SimFramework::Mask<Eigen::Vector3f, float> vec;

    // Write input values
    in.Write({1.5, 3.5, 5.5});
    out1.Write(0.f);
    out2.Write(0.f);
    out3.Write(0.f);

    // Only output to out1
    vec.Configure(&in, {&out2}, {1});
    vec.Update();
    ASSERT_FLOAT_EQ(out1.Read(), 0.f);
    ASSERT_FLOAT_EQ(out2.Read(), 3.5);
    ASSERT_FLOAT_EQ(out3.Read(), 0.f);
}

TEST(Mask, Vec32Out_1) {
    // Input Signal
    SimFramework::Signal<Eigen::Vector3f> in;

    // Output Signals
    SimFramework::Signal<float> out1;
    SimFramework::Signal<float> out2;
    SimFramework::Signal<float> out3;

    // Construct block
    SimFramework::Mask<Eigen::Vector3f, float> vec;

    // Write input values
    in.Write({1.5, 3.5, 5.5});
    out1.Write(0.f);
    out2.Write(0.f);
    out3.Write(0.f);

    // Only output to out1
    vec.Configure(&in, {&out2, &out3}, {2, 0});
    vec.Update();
    ASSERT_FLOAT_EQ(out1.Read(), 0.f);
    ASSERT_FLOAT_EQ(out2.Read(), 5.5);
    ASSERT_FLOAT_EQ(out3.Read(), 1.5);
}