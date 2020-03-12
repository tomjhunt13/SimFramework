#include <vector>
#include <iostream>
#include "gtest/gtest.h"
#include "Eigen/Dense"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

TEST(Constant, testFloat) {
    // Objects
    SimFramework::Signal<float> out;
    SimFramework::ConstantBlock<float> block;
    block.Configure(&out, 1.3);

    // Test
    block.Initialise(0.f);
    ASSERT_FLOAT_EQ(out.Read(), 1.3);
}


TEST(Input, testFloat) {
    // Objects
    SimFramework::Signal<float> out;
    SimFramework::Input<float> inBlock;
    inBlock.Configure(&out, 0.f);

    // Test
    inBlock.Initialise(0.f);
    ASSERT_FLOAT_EQ(out.Read(), 0.f);

    inBlock.WriteValue(1.5);
    inBlock.Update(0.f);
    ASSERT_FLOAT_EQ(out.Read(), 1.5);
}


TEST(StateSpace, PureIntegrator1) {
    // 1 input, 1 state and 1 output

    // Input Signal
    SimFramework::Signal<Eigen::Vector<float, 1>> in;

    // Output Signal
    SimFramework::Signal<Eigen::Vector<float, 1>> out;

    // Construct block
    SimFramework::StateSpace<Eigen::Vector<float, 1>, Eigen::Vector<float, 1>, 1, 1, 1> SS;
    SS.Configure(&in, &out);

    // Configure SS as pure integrator
    Eigen::Matrix<float, 1, 1> A;
    A << 0.f;

    Eigen::Matrix<float, 1, 1> B;
    B << 1.f;

    Eigen::Matrix<float, 1, 1> C;
    C << 1.f;

    Eigen::Matrix<float, 1, 1> D;
    D << 0.f;

    Eigen::Vector<float, 1> init;
    init << 1.f;

    SS.SetMatrices(A, B, C, D);
    SS.SetInitialConditions(init);

    // Write input values
    Eigen::Vector<float, 1> grad;
    grad << 2.f;
    in.Write(grad);

    // Test
    SS.Initialise(0.f);
    SS.ReadInputs();
    SS.Update(3.f);
    Eigen::Vector<float, 1> expected;
    expected << 7.f;
    Eigen::Vector<float, 1> actual = out.Read();
    ASSERT_FLOAT_EQ(expected[0], actual[0]);
}

TEST(StateSpace, PureIntegrator2) {
    // 2 inputs, 1 state and 1 output

    // Input Signal
    SimFramework::Signal<Eigen::Vector<float, 2>> in;

    // Output Signal
    SimFramework::Signal<Eigen::Vector<float, 1>> out;

    // Construct block
    SimFramework::StateSpace<Eigen::Vector<float, 2>, Eigen::Vector<float, 1>, 2, 1, 1> SS;
    SS.Configure(&in, &out);

    // Configure SS as pure integrator
    Eigen::Matrix<float, 1, 1> A;
    A << 0.f;

    Eigen::Matrix<float, 1, 2> B;
    B << 1.f, 1.f;

    Eigen::Matrix<float, 1, 1> C;
    C << 1.f;

    Eigen::Matrix<float, 1, 2> D;
    D << 0.f, 0.f;

    Eigen::Vector<float, 1> init;
    init << 1.f;

    SS.SetMatrices(A, B, C, D);
    SS.SetInitialConditions(init);

    // Write input values
    Eigen::Vector<float, 2> grad;
    grad << 2.f, 2.f;
    in.Write(grad);

    // Test
    SS.Initialise(0.f);
    SS.ReadInputs();
    SS.Update(3.f);
    Eigen::Vector<float, 1> expected;
    expected << 13.f;
    Eigen::Vector<float, 1> actual = out.Read();
    ASSERT_FLOAT_EQ(expected[0], actual[0]);
}

TEST(StateSpace, PureIntegrator3) {
    // 1 input, 2 states and 1 output

    // Input Signal
    SimFramework::Signal<Eigen::Vector<float, 1>> in;

    // Output Signal
    SimFramework::Signal<Eigen::Vector<float, 1>> out;

    // Construct block
    SimFramework::StateSpace<Eigen::Vector<float, 1>, Eigen::Vector<float, 1>, 1, 2, 1> SS;
    SS.Configure(&in, &out);

    // Configure SS as pure integrator
    Eigen::Matrix<float, 2, 2> A;
    A << 0.f, 0.f, 0.f, 0.f;

    Eigen::Matrix<float, 2, 1> B;
    B << 1.f, 1.f;

    Eigen::Matrix<float, 1, 2> C;
    C << 1.f, 1.f;

    Eigen::Matrix<float, 1, 1> D;
    D << 0.f;

    Eigen::Vector<float, 2> init;
    init << 1.f, 1.f;

    SS.SetMatrices(A, B, C, D);
    SS.SetInitialConditions(init);

    // Write input values
    Eigen::Vector<float, 1> grad;
    grad << 2.f;
    in.Write(grad);

    // Test
    SS.Initialise(0.f);
    SS.ReadInputs();
    SS.Update(3.f);
    Eigen::Vector<float, 1> expected;
    expected << 14.f;
    Eigen::Vector<float, 1> actual = out.Read();
    ASSERT_FLOAT_EQ(expected[0], actual[0]);
}

TEST(StateSpace, PureIntegrator4) {
    // 1 input, 1 state and 2 outputs

    // Input Signal
    SimFramework::Signal<Eigen::Vector<float, 1>> in;

    // Output Signal
    SimFramework::Signal<Eigen::Vector<float, 2>> out;

    // Construct block
    SimFramework::StateSpace<Eigen::Vector<float, 1>, Eigen::Vector<float, 2>, 1, 1, 2> SS;
    SS.Configure(&in, &out);

    // Configure SS as pure integrator
    Eigen::Matrix<float, 1, 1> A;
    A << 0.f;

    Eigen::Matrix<float, 1, 1> B;
    B << 1.f;

    Eigen::Matrix<float, 2, 1> C;
    C << 1.f, 1.f;

    Eigen::Matrix<float, 2, 1> D;
    D << 0.f, 0.f;

    Eigen::Vector<float, 1> init;
    init << 1.f;

    SS.SetMatrices(A, B, C, D);
    SS.SetInitialConditions(init);

    // Write input values
    Eigen::Vector<float, 1> grad;
    grad << 2.f;
    in.Write(grad);

    // Test
    SS.Initialise(0.f);
    SS.ReadInputs();
    SS.Update(3.f);
    Eigen::Vector<float, 2> expected;
    expected << 7.f, 7.f;
    Eigen::Vector<float, 2> actual = out.Read();
    ASSERT_FLOAT_EQ(expected[0], actual[0]);
    ASSERT_FLOAT_EQ(expected[1], actual[1]);
}

TEST(StateSpace, PureIntegrator5) {
    // 1 input, 2 states and 2 outputs

    // Input Signal
    SimFramework::Signal<Eigen::Vector<float, 1>> in;

    // Output Signal
    SimFramework::Signal<Eigen::Vector<float, 2>> out;

    // Construct block
    SimFramework::StateSpace<Eigen::Vector<float, 1>, Eigen::Vector<float, 2>, 1, 2, 2> SS;
    SS.Configure(&in, &out);

    // Configure SS as pure integrator
    Eigen::Matrix<float, 2, 2> A;
    A << 0.f, 0.f, 0.f, 0.f;

    Eigen::Matrix<float, 2, 1> B;
    B << 1.f, 0.f;

    Eigen::Matrix<float, 2, 2> C;
    C << 1.f, 0.f, 0.f, 0.f;

    Eigen::Matrix<float, 2, 1> D;
    D << 0.f, 0.f;

    Eigen::Vector<float, 2> init;
    init << 1.f, 1.f;

    SS.SetMatrices(A, B, C, D);
    SS.SetInitialConditions(init);

    // Write input values
    Eigen::Vector<float, 1> grad;
    grad << 2.f;
    in.Write(grad);

    // Test
    SS.Initialise(0.f);
    SS.ReadInputs();
    SS.Update(3.f);
    Eigen::Vector<float, 2> expected;
    expected << 7.f, 0.f;
    Eigen::Vector<float, 2> actual = out.Read();
    ASSERT_FLOAT_EQ(expected[0], actual[0]);
    ASSERT_FLOAT_EQ(expected[1], actual[1]);
}

TEST(StateSpace, PureIntegrator6) {
    // 2 inputs, 2 states and 2 outputs

    // Input Signal
    SimFramework::Signal<Eigen::Vector<float, 2>> in;

    // Output Signal
    SimFramework::Signal<Eigen::Vector<float, 2>> out;

    // Construct block
    SimFramework::StateSpace<Eigen::Vector<float, 2>, Eigen::Vector<float, 2>, 2, 2, 2> SS;
    SS.Configure(&in, &out);

    // Configure SS as pure integrator
    Eigen::Matrix<float, 2, 2> A;
    A << 0.f, 0.f, 0.f, 0.f;

    Eigen::Matrix<float, 2, 2> B;
    B << 1.f, 0.f, 0.f, 1.f;

    Eigen::Matrix<float, 2, 2> C;
    C << 1.f, 0.f, 0.f, 1.f;

    Eigen::Matrix<float, 2, 2> D;
    D << 0.f, 0.f, 0.f, 0.f;

    Eigen::Vector<float, 2> init;
    init << 1.f, 1.f;

    SS.SetMatrices(A, B, C, D);
    SS.SetInitialConditions(init);

    // Write input values
    Eigen::Vector<float, 2> grad;
    grad << 2.f, 3.f;
    in.Write(grad);

    // Test
    SS.Initialise(0.f);
    SS.ReadInputs();
    SS.Update(3.f);
    Eigen::Vector<float, 2> expected;
    expected << 7.f, 10.f;
    Eigen::Vector<float, 2> actual = out.Read();
    ASSERT_FLOAT_EQ(expected[0], actual[0]);
    ASSERT_FLOAT_EQ(expected[1], actual[1]);
}


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

TEST(Gain, Float_X_Vec) {
    // Input Signal
    SimFramework::Signal<Eigen::Vector3f> in;

    // Output Signal
    SimFramework::Signal<Eigen::Vector3f> out;

    // Construct block
    SimFramework::Gain<Eigen::Vector3f, Eigen::Vector3f, float> gain;

    // Configure gain
    gain.Configure(&in, &out, 2.f);

    // Write input values
    in.Write({1.5, 3.5, 5.5});

    // Test
    gain.Update();
    Eigen::Vector3f expected = {3.f, 7.f, 11.f};
    ASSERT_TRUE(out.Read() == expected);
}

TEST(Gain, Mat2x2_X_Vec2) {
    // Input Signal
    SimFramework::Signal<Eigen::Vector2f> in;

    // Output Signal
    SimFramework::Signal<Eigen::Vector2f> out;

    // Construct block
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f, Eigen::Matrix<float, 2, 2>> gain;

    // Configure gain
    Eigen::Matrix<float, 2, 2> gainMatrix;
    gainMatrix << 2.f, 2.f, 2.f, 2.f;
    gain.Configure(&in, &out, gainMatrix);

    // Write input values
    in.Write({1.5, 2.5});

    // Test
    gain.Update();
    Eigen::Vector2f expected = {8.f, 8.f};
    ASSERT_TRUE(out.Read() == expected);
}

TEST(Gain, Mat1x2_X_Vec2) {
    // Input Signal
    SimFramework::Signal<Eigen::Vector2f> in;

    // Output Signal
    SimFramework::Signal<Eigen::Vector<float, 1>> out;

    // Construct block
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector<float, 1>, Eigen::Matrix<float, 1, 2>> gain;

    // Configure gain
    Eigen::Matrix<float, 1, 2> gainMatrix;
    gainMatrix << 2.f, 2.f;
    gain.Configure(&in, &out, gainMatrix);

    // Write input values
    in.Write({1.5, 2.5});

    // Test
    gain.Update();
    Eigen::Vector<float, 1> expected;
    expected << 8.f;
    ASSERT_TRUE(out.Read() == expected);
}


TEST(LookupTable2D, test1) {
    // Input Signals
    SimFramework::Signal<float> inX;
    SimFramework::Signal<float> inY;

    // Output Signal
    SimFramework::Signal<float> out;

    // Construct block
    SimFramework::LookupTable2D block;
    block.Configure(&inX, &inY, &out);

    // Set up table
    SimFramework::Table3D tab;
    tab.x = {1.f, 2.f};
    tab.y = {1.f, 2.f};
    tab.z = {{0.f, 0.f}, {1.f, 1.f}};
    block.SetTable(tab);

    // Test
    inX.Write(1.5);
    inY.Write(1.f);
    block.Update();
    ASSERT_FLOAT_EQ(out.Read(), 0.f);

    inY.Write(1.5);
    block.Update();
    ASSERT_FLOAT_EQ(out.Read(), 0.5);
}


TEST(Output, FloatSig) {
    // Input Signal
    SimFramework::Signal<float> in;

    // Construct block
    SimFramework::Output<float> block;
    block.Configure(&in, 0.f);

    // Test
    ASSERT_FLOAT_EQ(block.ReadValue(), 0.f);
    in.Write(1.f);
    block.Update(0.f);
    ASSERT_FLOAT_EQ(block.ReadValue(), 1.f);
}