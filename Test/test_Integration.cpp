#include <cmath>
#include "gtest/gtest.h"
#include "Eigen/Dense"
#include "SimFramework/Framework.h"

// f(t, x) = 4x + c
class Linear : public SimFramework::Integrable<float>
{
public:
    float Derivative(float t, const float& x) override
    {
        return 4.f;
    };
};

// f(t, x) = 3x^2 + 2x + c
class Quadratic : public SimFramework::Integrable<float>
{
public:

    float Derivative(float t, const float& x) override
    {
        return 2 * 3 * x + 2;
    };
};

// f1(t, x) = 2x + c
// f2(t, x) = 3x + c
class LinearVector : public SimFramework::Integrable<Eigen::Vector2f>
{
public:
    Eigen::Vector2f Derivative(float t, const Eigen::Vector2f& x) override
    {
        return {4.f, 3.f};
    };
};




TEST(Euler, LinearFloat)
{
    Linear eqn;
    ASSERT_FLOAT_EQ(SimFramework::ForwardEuler::Step<float>(&eqn, 1.f, 0.f, 1.f), 5.f);
    ASSERT_FLOAT_EQ(SimFramework::ForwardEuler::Step<float>(&eqn, 0.5, 0.f, 2.f), 4.f);
}

TEST(Euler, QuadraticFloat)
{
    Quadratic eqn;
    float tol = 0.001;

    float res1 = SimFramework::ForwardEuler::Step<float>(&eqn, 0.001, 0.f, 1.f);
    ASSERT_LE(std::abs(res1 - 1.008003), tol);

    float res2 = SimFramework::RK4::Step<float>(&eqn, 0.001, 0.f, 2.f);
    ASSERT_LE(std::abs(res2 - 2.014003), tol);
}

TEST(Euler, LinearVector)
{
    LinearVector eqn;
    Eigen::Vector2f res1 = SimFramework::ForwardEuler::Step<Eigen::Vector2f>(&eqn, 1.f, 0.f, {1.f, 1.f});
    ASSERT_FLOAT_EQ(res1[0], 5.f);
    ASSERT_FLOAT_EQ(res1[1], 4.f);
}

TEST(RK4, LinearFloat)
{
    Linear eqn;
    float res = SimFramework::RK4::Step<float>(&eqn, 1.f, 0.f, 1.f);
    ASSERT_FLOAT_EQ(SimFramework::RK4::Step<float>(&eqn, 1.f, 0.f, 1.f), 5.f);
    ASSERT_FLOAT_EQ(SimFramework::RK4::Step<float>(&eqn, 0.5, 0.f, 2.f), 4.f);
}

TEST(RK4, QuadraticFloat)
{
    Quadratic eqn;
    float tol = 0.001;

    float res1 = SimFramework::RK4::Step<float>(&eqn, 0.001, 0.f, 1.f);
    ASSERT_LE(std::abs(res1 - 1.008003), tol);

    float res2 = SimFramework::RK4::Step<float>(&eqn, 0.001, 0.f, 2.f);
    ASSERT_LE(std::abs(res2 - 2.014003), tol);
}

TEST(RK4, LinearVector)
{
    LinearVector eqn;
    Eigen::Vector2f res1 = SimFramework::RK4::Step<Eigen::Vector2f>(&eqn, 1.f, 0.f, {1.f, 1.f});
    ASSERT_FLOAT_EQ(res1[0], 5.f);
    ASSERT_FLOAT_EQ(res1[1], 4.f);
}