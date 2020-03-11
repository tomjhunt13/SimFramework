#include <vector>
#include "gtest/gtest.h"
#include "SimFramework/Interpolation.h"

TEST(LinearInterp, P1P2Ordered)
{
    SimFramework::Coord2D P1 = {1.5, 4.5};
    SimFramework::Coord2D P2 = {4.f, -1.5};

    ASSERT_FLOAT_EQ(SimFramework::LinearInterp(-1.f, P1, P2), 10.5);
    ASSERT_FLOAT_EQ(SimFramework::LinearInterp(0.f, P1, P2), 8.1);
    ASSERT_FLOAT_EQ(SimFramework::LinearInterp(1.5, P1, P2), 4.5);
    ASSERT_FLOAT_EQ(SimFramework::LinearInterp(3.f, P1, P2), 0.9);
    ASSERT_FLOAT_EQ(SimFramework::LinearInterp(4.f, P1, P2), -1.5);
    ASSERT_FLOAT_EQ(SimFramework::LinearInterp(7.f, P1, P2), -8.7);
}

TEST(LinearInterp, P1P2Reversed)
{
    SimFramework::Coord2D P2 = {1.5, 4.5};
    SimFramework::Coord2D P1 = {4.f, -1.5};

    ASSERT_FLOAT_EQ(SimFramework::LinearInterp(-1.f, P1, P2), 10.5);
    ASSERT_FLOAT_EQ(SimFramework::LinearInterp(0.f, P1, P2), 8.1);
    ASSERT_FLOAT_EQ(SimFramework::LinearInterp(1.5, P1, P2), 4.5);
    ASSERT_FLOAT_EQ(SimFramework::LinearInterp(3.f, P1, P2), 0.9);
    ASSERT_FLOAT_EQ(SimFramework::LinearInterp(4.f, P1, P2), -1.5);
    ASSERT_FLOAT_EQ(SimFramework::LinearInterp(7.f, P1, P2), -8.7);
}

TEST(BilinearInterp, Flat)
{
    SimFramework::Coord3D P11 = {1.f, 1.f, 2.f};
    SimFramework::Coord3D P12 = {3.f, 1.f, 2.f};
    SimFramework::Coord3D P21 = {1.f, 2.f, 2.f};
    SimFramework::Coord3D P22 = {3.f, 2.f, 2.f};

    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({1.f, 1.f}, P11, P21, P12, P22), 2.f);
    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({2.f, 1.5}, P11, P21, P12, P22), 2.f);
    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({-1.f, -0.5}, P11, P21, P12, P22), 2.f);
    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({3.f, 2.f}, P11, P21, P12, P22), 2.f);
}

TEST(BilinearInterp, SlopedIn1D)
{
    SimFramework::Coord3D P11 = {1.f, 1.f, 2.f};
    SimFramework::Coord3D P12 = {3.f, 1.f, 4.f};
    SimFramework::Coord3D P21 = {1.f, 2.f, 2.f};
    SimFramework::Coord3D P22 = {3.f, 2.f, 4.f};

    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({1.f, 1.f}, P11, P21, P12, P22), 2.f);
    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({3.f, 1.5}, P11, P21, P12, P22), 4.f);
    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({3.f, -0.5}, P11, P21, P12, P22), 4.f);
    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({2.f, 2.f}, P11, P21, P12, P22), 3.f);
    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({0.f, 2.f}, P11, P21, P12, P22), 1.f);
}

TEST(BilinearInterp, SlopedIn2D)
{
    SimFramework::Coord3D P11 = {1.f, 1.f, 2.f};
    SimFramework::Coord3D P12 = {3.f, 1.f, 3.f};
    SimFramework::Coord3D P21 = {1.f, 3.f, 3.f};
    SimFramework::Coord3D P22 = {3.f, 3.f, 4.f};

    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({1.f, 1.f}, P11, P21, P12, P22), 2.f);
    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({2.f, 2.f}, P11, P21, P12, P22), 3.f);
    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({2.f, 3}, P11, P21, P12, P22), 3.5);
    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({2.f, 1.f}, P11, P21, P12, P22), 2.5);
    ASSERT_FLOAT_EQ(SimFramework::BilinearInterp({3.f, 3.f}, P11, P21, P12, P22), 4.f);
}

TEST(NearestIndices, Test1) {
    std::vector<float> array = {-10.5, -2.3, 0.f, 0.1, 3.3, 7.f};

    // Cycle through values in between and including elements

    std::vector<int> expected = {0, 1};
    std::vector<int> actual = SimFramework::Internal::NearestIndices(array, -20);
    ASSERT_EQ(expected, actual);

    expected = {0, 1};
    actual = SimFramework::Internal::NearestIndices(array, -10.5);
    ASSERT_EQ(expected, actual);

    expected = {0, 1};
    actual = SimFramework::Internal::NearestIndices(array, -5.4);
    ASSERT_EQ(expected, actual);

    expected = {1, 2};
    actual = SimFramework::Internal::NearestIndices(array, -2.3);
    ASSERT_EQ(expected, actual);

    expected = {1, 2};
    actual = SimFramework::Internal::NearestIndices(array, -0.3);
    ASSERT_EQ(expected, actual);

    expected = {2, 3};
    actual = SimFramework::Internal::NearestIndices(array, 0.f);
    ASSERT_EQ(expected, actual);

    expected = {2, 3};
    actual = SimFramework::Internal::NearestIndices(array, 0.05);
    ASSERT_EQ(expected, actual);

    expected = {3, 4};
    actual = SimFramework::Internal::NearestIndices(array, 1.5);
    ASSERT_EQ(expected, actual);

    expected = {4, 5};
    actual = SimFramework::Internal::NearestIndices(array, 3.3);
    ASSERT_EQ(expected, actual);

    expected = {4, 5};
    actual = SimFramework::Internal::NearestIndices(array, 5.3);
    ASSERT_EQ(expected, actual);

    expected = {4, 5};
    actual = SimFramework::Internal::NearestIndices(array, 7.f);
    ASSERT_EQ(expected, actual);

    expected = {4, 5};
    actual = SimFramework::Internal::NearestIndices(array, 10.3);
    ASSERT_EQ(expected, actual);
}


TEST(InterpTable3D, Test1) {
    SimFramework::Table3D tab;
    tab.x = {1.f, 2.f, 3.f};
    tab.y = {1.f, 2.f, 3.f, 4.f};
    tab.z = {{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {2.f, 2.f, 0.f}, {1.f, 1.f, 0.f}};

    ASSERT_FLOAT_EQ(SimFramework::InterpTable3D(tab, {0.f, 0.f}), 0.f);
    ASSERT_FLOAT_EQ(SimFramework::InterpTable3D(tab, {1.f, 1.f}), 0.f);
    ASSERT_FLOAT_EQ(SimFramework::InterpTable3D(tab, {1.f, 3.f}), 2.f);
    ASSERT_FLOAT_EQ(SimFramework::InterpTable3D(tab, {1.f, 4.f}), 1.f);
    ASSERT_FLOAT_EQ(SimFramework::InterpTable3D(tab, {1.5, 2.5}), 1.f);
    ASSERT_FLOAT_EQ(SimFramework::InterpTable3D(tab, {2.5, 3.f}), 1.f);
    ASSERT_FLOAT_EQ(SimFramework::InterpTable3D(tab, {1.5, 3.5}), 1.5);
    ASSERT_FLOAT_EQ(SimFramework::InterpTable3D(tab, {-1.f, 2.5}), 1.f);
}