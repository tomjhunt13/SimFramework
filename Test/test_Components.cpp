#include <vector>
#include <iostream>
#include "gtest/gtest.h"
#include "Eigen/Dense"
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

class LinearTriggerFunction : public SimFramework::TriggerFunction {
public:
    LinearTriggerFunction(float defaultVal, float t_end)
    {
        this->m_Default = defaultVal;
        this->t_end = t_end;
    }

    float Evaluate(float t) override
    {
        return 1.f * (this->t_end - t) / (this->t_end);
    };
};


TEST(Constant, testFloat) {
    // Objects
    SimFramework::ConstantBlock<float> block;
    block.Configure(1.3);
    const SimFramework::Signal<float>* out = block.OutSignal();


    // Test
    block.Initialise(0.f);
    ASSERT_FLOAT_EQ(out->Read(), 1.3);
}


TEST(Input, testFloat) {
    // Objects
    SimFramework::Input<float> inBlock;
    inBlock.Configure(0.f);
    const SimFramework::Signal<float>* out = inBlock.OutSignal();

    // Test
    inBlock.Initialise(0.f);
    ASSERT_FLOAT_EQ(out->Read(), 0.f);

    inBlock.WriteValue(1.5);
    inBlock.Update(0.f);
    ASSERT_FLOAT_EQ(out->Read(), 1.5);
}


TEST(TriggerFunction, NoTrigger) {
    // Construct block
    LinearTriggerFunction block(1.f, 5.f);
    block.Initialise(0.f);

    // Output Signal
    const SimFramework::Signal<float>* out = block.OutSignal();

    // Test
    block.Update(1.f);
    ASSERT_FLOAT_EQ(out->Read(), 1.f);
    block.Update(1.f);
    ASSERT_FLOAT_EQ(out->Read(), 1.f);
}

TEST(TriggerFunction, NormalTrigger) {
    // Construct block
    LinearTriggerFunction block(1.f, 5.f);
    block.Initialise(0.f);

    // Output Signal
    const SimFramework::Signal<float>* out = block.OutSignal();

    // Test
    block.Update(1.f);
    ASSERT_FLOAT_EQ(out->Read(), 1.f);

    block.Trigger();
    block.Update(1.f);
    ASSERT_FLOAT_EQ(out->Read(), 0.8);
    block.Update(1.f);
    ASSERT_FLOAT_EQ(out->Read(), 0.6);
    block.Update(2.f);
    ASSERT_FLOAT_EQ(out->Read(), 0.2);
    block.Update(1.f);
    ASSERT_FLOAT_EQ(out->Read(), 0.0);
    block.Update(0.01);
    ASSERT_FLOAT_EQ(out->Read(), 1.f);
}

TEST(TriggerFunction, Duplicate) {
    // Construct block
    LinearTriggerFunction block(1.f, 5.f);
    block.Initialise(0.f);

    // Output Signal
    const SimFramework::Signal<float>* out = block.OutSignal();

    // Test
    block.Update(1.f);
    ASSERT_FLOAT_EQ(out->Read(), 1.f);

    block.Trigger();
    block.Update(1.f);
    ASSERT_FLOAT_EQ(out->Read(), 0.8);
    block.Update(1.f);
    ASSERT_FLOAT_EQ(out->Read(), 0.6);

    block.Trigger();
    block.Update(2.f);
    ASSERT_FLOAT_EQ(out->Read(), 0.6);
    block.Update(1.f);
    ASSERT_FLOAT_EQ(out->Read(), 0.4);
    block.Update(2.01);
    ASSERT_FLOAT_EQ(out->Read(), 1.f);
}


TEST(StateSpace, PureIntegrator1) {
    // 1 input, 1 state and 1 output

    // Input Signal
    SimFramework::Signal<Eigen::Vector<float, 1>> in;

    // Construct block
    SimFramework::StateSpace<Eigen::Vector<float, 1>, Eigen::Vector<float, 1>, 1, 1, 1> SS;
    SS.Configure(&in);

    // Output Signal
    const SimFramework::Signal<Eigen::Vector<float, 1>>* out = SS.OutSignal();

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
    Eigen::Vector<float, 1> actual = out->Read();
    ASSERT_FLOAT_EQ(expected[0], actual[0]);
}

TEST(StateSpace, PureIntegrator2) {
    // 2 inputs, 1 state and 1 output

    // Input Signal
    SimFramework::Signal<Eigen::Vector<float, 2>> in;

    // Construct block
    SimFramework::StateSpace<Eigen::Vector<float, 2>, Eigen::Vector<float, 1>, 2, 1, 1> SS;
    SS.Configure(&in);

    // Output Signal
    const SimFramework::Signal<Eigen::Vector<float, 1>>* out = SS.OutSignal();

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
    Eigen::Vector<float, 1> actual = out->Read();
    ASSERT_FLOAT_EQ(expected[0], actual[0]);
}

TEST(StateSpace, PureIntegrator3) {
    // 1 input, 2 states and 1 output

    // Input Signal
    SimFramework::Signal<Eigen::Vector<float, 1>> in;

    // Construct block
    SimFramework::StateSpace<Eigen::Vector<float, 1>, Eigen::Vector<float, 1>, 1, 2, 1> SS;
    SS.Configure(&in);

    // Output Signal
    const SimFramework::Signal<Eigen::Vector<float, 1>>* out = SS.OutSignal();

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
    Eigen::Vector<float, 1> actual = out->Read();
    ASSERT_FLOAT_EQ(expected[0], actual[0]);
}

TEST(StateSpace, PureIntegrator4) {
    // 1 input, 1 state and 2 outputs

    // Input Signal
    SimFramework::Signal<Eigen::Vector<float, 1>> in;

    // Construct block
    SimFramework::StateSpace<Eigen::Vector<float, 1>, Eigen::Vector<float, 2>, 1, 1, 2> SS;
    SS.Configure(&in);

    // Output Signal
    const SimFramework::Signal<Eigen::Vector<float, 2>>* out = SS.OutSignal();

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
    Eigen::Vector<float, 2> actual = out->Read();
    ASSERT_FLOAT_EQ(expected[0], actual[0]);
    ASSERT_FLOAT_EQ(expected[1], actual[1]);
}

TEST(StateSpace, PureIntegrator5) {
    // 1 input, 2 states and 2 outputs

    // Input Signal
    SimFramework::Signal<Eigen::Vector<float, 1>> in;

    // Construct block
    SimFramework::StateSpace<Eigen::Vector<float, 1>, Eigen::Vector<float, 2>, 1, 2, 2> SS;
    SS.Configure(&in);

    // Output Signal
    const SimFramework::Signal<Eigen::Vector<float, 2>>* out = SS.OutSignal();

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
    Eigen::Vector<float, 2> actual = out->Read();
    ASSERT_FLOAT_EQ(expected[0], actual[0]);
    ASSERT_FLOAT_EQ(expected[1], actual[1]);
}

TEST(StateSpace, PureIntegrator6) {
    // 2 inputs, 2 states and 2 outputs

    // Input Signal
    SimFramework::Signal<Eigen::Vector<float, 2>> in;

    // Construct block
    SimFramework::StateSpace<Eigen::Vector<float, 2>, Eigen::Vector<float, 2>, 2, 2, 2> SS;
    SS.Configure(&in);

    // Output Signal
    const SimFramework::Signal<Eigen::Vector<float, 2>>* out = SS.OutSignal();

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
    Eigen::Vector<float, 2> actual = out->Read();
    ASSERT_FLOAT_EQ(expected[0], actual[0]);
    ASSERT_FLOAT_EQ(expected[1], actual[1]);
}


TEST(SummingJunction, testFloat) {
    // Input Signals
    SimFramework::Signal<float> in1;
    SimFramework::Signal<float> in2;
    SimFramework::Signal<float> in3;

    // Construct block
    SimFramework::SummingJunction<float> sum;

    // Configure as [+++]
    sum.Configure({&in1, &in2, &in3}, {1.f, 1.f, 1.f});

    // Output Signal
    const SimFramework::Signal<float>* out = sum.OutSignal();

    // Write input values
    in1.Write(1.5);
    in2.Write(3.5);
    in3.Write(4.5);

    // Test
    sum.Update();
    ASSERT_FLOAT_EQ(out->Read(), 9.5);

    // Configure as [+--]
    sum.Configure({&in1, &in2, &in3}, {1.f, -1.f, -1.f});

    // Test
    sum.Update();
    ASSERT_FLOAT_EQ(out->Read(), -6.5);
}


TEST(Vectorise, Vec2) {
    // Input Signals
    SimFramework::Signal<float> in1;
    SimFramework::Signal<float> in2;

    // Construct block
    SimFramework::Vectorise<float, Eigen::Vector2f> vec;

    // Configure as [+++]
    vec.Configure({&in1, &in2});

    // Output Signal
    const SimFramework::Signal<Eigen::Vector2f>* out = vec.OutSignal();

    // Write input values
    in1.Write(1.5);
    in2.Write(3.5);

    // Test
    Eigen::Vector2f expected = {1.5, 3.5};
    vec.Update();
    ASSERT_TRUE(out->Read() == expected);
}

TEST(Vectorise, Vec3) {
    // Input Signals
    SimFramework::Signal<float> in1;
    SimFramework::Signal<float> in2;
    SimFramework::Signal<float> in3;

    // Construct block
    SimFramework::Vectorise<float, Eigen::Vector3f> vec;

    // Configure as [+++]
    vec.Configure({&in1, &in2, &in3});

    // Output Signal
    const SimFramework::Signal<Eigen::Vector3f>* out = vec.OutSignal();

    // Write input values
    in1.Write(1.5);
    in2.Write(3.5);
    in3.Write(5.5);

    // Test
    Eigen::Vector3f expected = {1.5, 3.5, 5.5};
    vec.Update();
    ASSERT_TRUE(out->Read() == expected);
}


TEST(Mask, Vec3_1Out_1) {
    // Input Signal
    SimFramework::Signal<Eigen::Vector3f> in;

    // Construct block
    SimFramework::Mask<Eigen::Vector3f, float> mask;

    // Write input values
    in.Write({1.5, 3.5, 5.5});

    // Only output to out1
    mask.Configure(&in, {0});
    mask.Update();
    ASSERT_FLOAT_EQ(mask.OutSignal(0)->Read(), 1.5);
    ASSERT_FLOAT_EQ(mask.OutSignal(1)->Read(), 3.5);
    ASSERT_FLOAT_EQ(mask.OutSignal(2)->Read(), 5.5);
}

TEST(Gain, Float_X_Vec) {
    // Input Signal
    SimFramework::Signal<Eigen::Vector3f> in;

    // Construct block
    SimFramework::Gain<Eigen::Vector3f, Eigen::Vector3f, float> gain;

    // Output Signal
    const SimFramework::Signal<Eigen::Vector3f>* out = gain.OutSignal();

    // Configure gain
    gain.Configure(&in, 2.f);

    // Write input values
    in.Write({1.5, 3.5, 5.5});

    // Test
    gain.Update();
    Eigen::Vector3f expected = {3.f, 7.f, 11.f};
    ASSERT_TRUE(out->Read() == expected);
}

TEST(Gain, Mat2x2_X_Vec2) {
    // Input Signal
    SimFramework::Signal<Eigen::Vector2f> in;

    // Construct block
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f, Eigen::Matrix<float, 2, 2>> gain;

    // Output Signal
    const SimFramework::Signal<Eigen::Vector2f>* out = gain.OutSignal();

    // Configure gain
    Eigen::Matrix<float, 2, 2> gainMatrix;
    gainMatrix << 2.f, 2.f, 2.f, 2.f;
    gain.Configure(&in, gainMatrix);

    // Write input values
    in.Write({1.5, 2.5});

    // Test
    gain.Update();
    Eigen::Vector2f expected = {8.f, 8.f};
    ASSERT_TRUE(out->Read() == expected);
}

TEST(Gain, Mat1x2_X_Vec2) {
    // Input Signal
    SimFramework::Signal<Eigen::Vector2f> in;

    // Construct block
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector<float, 1>, Eigen::Matrix<float, 1, 2>> gain;

    // Output Signal
    const SimFramework::Signal<Eigen::Vector<float, 1>>* out = gain.OutSignal();

    // Configure gain
    Eigen::Matrix<float, 1, 2> gainMatrix;
    gainMatrix << 2.f, 2.f;
    gain.Configure(&in, gainMatrix);

    // Write input values
    in.Write({1.5, 2.5});

    // Test
    gain.Update();
    Eigen::Vector<float, 1> expected;
    expected << 8.f;
    ASSERT_TRUE(out->Read() == expected);
}


TEST(LookupTable2D, test1) {
    // Input Signals
    SimFramework::Signal<float> inX;
    SimFramework::Signal<float> inY;

    // Construct block
    SimFramework::LookupTable2D block;
    block.Configure(&inX, &inY);

    // Output Signal
    const SimFramework::Signal<float>* out = block.OutSignal();

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
    ASSERT_FLOAT_EQ(out->Read(), 0.f);

    inY.Write(1.5);
    block.Update();
    ASSERT_FLOAT_EQ(out->Read(), 0.5);
}


TEST(LinearBlend, test1) {
    // Input Signals
    SimFramework::Signal<Eigen::Vector3f> in1;
    SimFramework::Signal<Eigen::Vector3f> in2;
    SimFramework::Signal<float> inAlpha;

    // Construct block
    SimFramework::LinearBlend<Eigen::Vector3f> block;
    block.Configure(&in1, &in2, &inAlpha);

    // Output Signal
    const SimFramework::Signal<Eigen::Vector3f>* out = block.OutSignal();

    // Test
    Eigen::Vector3f vec1 = {4.f, 2.f, 0.f};
    Eigen::Vector3f vec2 = {0.f, 2.f, 4.f};
    in1.Write(vec1);
    in2.Write(vec2);

    inAlpha.Write(0.f);
    block.Update();
    Eigen::Vector3f expected = {4.f, 2.f, 0.f};
    ASSERT_TRUE(out->Read() == expected);

    inAlpha.Write(0.5);
    block.Update();
    expected = {2.f, 2.f, 2.f};
    ASSERT_TRUE(out->Read() == expected);

    inAlpha.Write(1.f);
    block.Update();
    expected = {0.f, 2.f, 4.f};
    ASSERT_TRUE(out->Read() == expected);
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