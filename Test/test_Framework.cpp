#include <vector>
#include <map>

#include "gtest/gtest.h"
#include "Eigen/Dense"

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

TEST(FunctionOutputs, TwoParallelTrees) {
    // Signals
    SimFramework::Signal<float> s1;
    SimFramework::Signal<float> s2;
    SimFramework::Signal<float> s3;
    SimFramework::Signal<float> s4;
    SimFramework::Signal<float> s5;
    SimFramework::Signal<float> s6;

    // Blocks
    SimFramework::Gain<float, float> a;
    SimFramework::Gain<float, float> b;
    SimFramework::Gain<float, float> c;
    SimFramework::Gain<float, float> d;

    a.Configure(&s1, &s2, 1);
    b.Configure(&s2, &s3, 1);
    c.Configure(&s4, &s5, 1);
    d.Configure(&s5, &s6, 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d};
    std::map<SimFramework::SignalBase*, SimFramework::Function*> outputMap = SimFramework::Internal::FunctionOutputs(funcs);

    // Assertions
    ASSERT_EQ(outputMap.size(), 4);
    ASSERT_TRUE(outputMap[&s2] == &a);
    ASSERT_TRUE(outputMap[&s3] == &b);
    ASSERT_TRUE(outputMap[&s5] == &c);
    ASSERT_TRUE(outputMap[&s6] == &d);
}

TEST(FunctionOutputs, SummingJunction) {
    // Signals
    SimFramework::Signal<float> s1;
    SimFramework::Signal<float> s2;
    SimFramework::Signal<float> s3;
    SimFramework::Signal<float> s4;
    SimFramework::Signal<float> s5;
    SimFramework::Signal<float> s6;
    SimFramework::Signal<float> s7;

    // Blocks
    SimFramework::Gain<float, float> a;
    SimFramework::Gain<float, float> b;
    SimFramework::SummingJunction<float> c;
    SimFramework::Gain<float, float> d;

    a.Configure(&s1, &s2, 1);
    b.Configure(&s3, &s4, 1);
    c.Configure({&s2, &s4, &s5}, &s6, {1.f, 1.f, 1.f});
    d.Configure(&s6, &s7, 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d};

    std::map<SimFramework::SignalBase*, SimFramework::Function*> outputMap = SimFramework::Internal::FunctionOutputs(funcs);

    // Assertions
    ASSERT_EQ(outputMap.size(), 4);
    ASSERT_TRUE(outputMap[&s2] == &a);
    ASSERT_TRUE(outputMap[&s4] == &b);
    ASSERT_TRUE(outputMap[&s6] == &c);
    ASSERT_TRUE(outputMap[&s7] == &d);
}

TEST(FunctionOutputs, Mask) {
    // Signals
    SimFramework::Signal<Eigen::Vector3f> s1;
    SimFramework::Signal<Eigen::Vector3f> s2;
    SimFramework::Signal<float> s3;
    SimFramework::Signal<float> s4;
    SimFramework::Signal<float> s5;
    SimFramework::Signal<float> s6;
    SimFramework::Signal<float> s7;

    // Blocks
    SimFramework::Gain<Eigen::Vector3f, Eigen::Vector3f> a;
    SimFramework::Mask<Eigen::Vector3f, float> b;
    SimFramework::Gain<float, float> c;
    SimFramework::Gain<float, float> d;

    a.Configure(&s1, &s2, 1);
    b.Configure(&s2, {&s3, &s5, &s7}, {0, 1, 2});
    c.Configure(&s3, &s4, 1);
    d.Configure(&s5, &s6, 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d};

    std::map<SimFramework::SignalBase*, SimFramework::Function*> outputMap = SimFramework::Internal::FunctionOutputs(funcs);

    // Assertions
    ASSERT_EQ(outputMap.size(), 6);
    ASSERT_TRUE(outputMap[&s2] == &a);
    ASSERT_TRUE(outputMap[&s3] == &b);
    ASSERT_TRUE(outputMap[&s4] == &c);
    ASSERT_TRUE(outputMap[&s5] == &b);
    ASSERT_TRUE(outputMap[&s6] == &d);
    ASSERT_TRUE(outputMap[&s7] == &b);
}


TEST(UnpackTree, TwoParallelTrees) {

    // Blocks
    SimFramework::Gain<float, float> a;
    SimFramework::Gain<float, float> b;
    SimFramework::Gain<float, float> c;
    SimFramework::Gain<float, float> d;

    SimFramework::Internal::FunctionTree ta = {&a};
    SimFramework::Internal::FunctionTree tb = {&b};
    SimFramework::Internal::FunctionTree tc = {&c};
    SimFramework::Internal::FunctionTree td = {&d};

    tb.children.push_back(&ta);
    td.children.push_back(&tc);

    std::vector<SimFramework::Function*> o1 = SimFramework::Internal::UnpackTree(tb);
    std::vector<SimFramework::Function*> o2 = SimFramework::Internal::UnpackTree(td);

    // Assertions
    ASSERT_EQ(o1.size(), 2);
    ASSERT_EQ(o2.size(), 2);
    ASSERT_EQ(o1[0], &a);
    ASSERT_EQ(o1[1], &b);
    ASSERT_EQ(o2[0], &c);
    ASSERT_EQ(o2[1], &d);
}

TEST(UnpackTree, SummingJunction) {

    // Blocks
    SimFramework::Gain<float, float> a;
    SimFramework::Gain<float, float> b;
    SimFramework::SummingJunction<float> c;
    SimFramework::Gain<float, float> d;

    SimFramework::Internal::FunctionTree ta = {&a};
    SimFramework::Internal::FunctionTree tb = {&b};
    SimFramework::Internal::FunctionTree tc = {&c};
    SimFramework::Internal::FunctionTree td = {&d};

    td.children.push_back(&tc);
    tc.children.push_back(&ta);
    tc.children.push_back(&tb);

    std::vector<SimFramework::Function*> o1 = SimFramework::Internal::UnpackTree(td);

    // Assertions
    ASSERT_EQ(o1.size(), 4);
    ASSERT_EQ(o1[3], &d);
    ASSERT_EQ(o1[2], &c);
    ASSERT_TRUE(o1[0] == &a || o1[0] == &b);
    ASSERT_TRUE(o1[1] == &a || o1[1] == &b);
}

TEST(UnpackTree, Mask) {

    // Blocks
    SimFramework::Gain<Eigen::Vector3f, Eigen::Vector3f> a;
    SimFramework::Mask<Eigen::Vector3f, float> b;
    SimFramework::Gain<float, float> c;
    SimFramework::Gain<float, float> d;

    SimFramework::Internal::FunctionTree ta = {&a};
    SimFramework::Internal::FunctionTree tb = {&b};
    SimFramework::Internal::FunctionTree tc = {&c};
    SimFramework::Internal::FunctionTree td = {&d};

    tc.children.push_back(&tb);
    td.children.push_back(&tb);
    tb.children.push_back(&ta);

    std::vector<SimFramework::Function*> o1 = SimFramework::Internal::UnpackTree(tc);
    std::vector<SimFramework::Function*> o2 = SimFramework::Internal::UnpackTree(td);

    // Assertions
    ASSERT_EQ(o1.size(), 3);
    ASSERT_EQ(o1[2], &c);
    ASSERT_EQ(o1[1], &b);
    ASSERT_EQ(o1[0], &a);

    ASSERT_EQ(o2.size(), 3);
    ASSERT_EQ(o2[2], &d);
    ASSERT_EQ(o2[1], &b);
    ASSERT_EQ(o2[0], &a);
}

//TEST(AssembleTree, TwoSerialTrees) {
//    // Signals
//    SimFramework::Signal<float> s1;
//    SimFramework::Signal<float> s2;
//    SimFramework::Signal<float> s3;
//    SimFramework::Signal<float> s4;
//    SimFramework::Signal<float> s5;
//    SimFramework::Signal<float> s6;
//
//    // Blocks
//    SimFramework::Gain<float, float> a;
//    SimFramework::Gain<float, float> b;
//    SimFramework::Gain<float, float> c;
//    SimFramework::Gain<float, float> d;
//
//    a.Configure(&s1, &s2, 1);
//    b.Configure(&s2, &s3, 1);
//    c.Configure(&s4, &s5, 1);
//    d.Configure(&s5, &s6, 1);
//
//
//    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d};
//    std::vector<SimFramework::Internal::FunctionTree> ft = SimFramework::Internal::AssembleTree(funcs);
//
//    // Assertions
//    ASSERT_EQ(ft.size(), 2);
//    ASSERT_TRUE(ft[0].block == &b || ft[0].block == &d);
//    ASSERT_TRUE(ft[1].block == &b || ft[1].block == &d);
//
//    if (ft[0].block == &b)
//    {
//        ASSERT_TRUE(ft[0].children[0]->block == &a);
//        ASSERT_TRUE(ft[1].children[0]->block == &c);
//    }
//
//    else if (ft[0].block == &d)
//    {
//        ASSERT_TRUE(ft[0].children[0]->block == &c);
//        ASSERT_TRUE(ft[1].children[0]->block == &a);
//    }
//}

//TEST(AssembleTree, SummingJunction) {
//    // Signals
//    SimFramework::Signal<float> s1;
//    SimFramework::Signal<float> s2;
//    SimFramework::Signal<float> s3;
//    SimFramework::Signal<float> s4;
//    SimFramework::Signal<float> s5;
//    SimFramework::Signal<float> s6;
//    SimFramework::Signal<float> s7;
//
//    // Blocks
//    SimFramework::Gain<float, float> a;
//    SimFramework::Gain<float, float> b;
//    SimFramework::SummingJunction<float> c;
//    SimFramework::Gain<float, float> d;
//
//    a.Configure(&s1, &s2, 1);
//    b.Configure(&s3, &s4, 1);
//    c.Configure({&s2, &s4, &s5}, &s6, {1.f, 1.f, 1.f});
//    d.Configure(&s6, &s7, 1);
//
//    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d};
//    std::vector<SimFramework::Internal::FunctionTree> ft = SimFramework::Internal::AssembleTree(funcs);
//
//    // Assertions
//    ASSERT_EQ(ft.size(), 1);
//    ASSERT_TRUE(ft[0].block == &d);
//    ASSERT_TRUE(ft[0].children[0]->block == &c);
//    ASSERT_EQ(ft[0].children[0]->children.size(), 2);
//}