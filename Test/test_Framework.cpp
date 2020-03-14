#include <vector>
#include <map>

#include "gtest/gtest.h"
#include "Eigen/Dense"

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

TEST(FunctionInputs, TwoParallelTrees) {
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

    std::map<SimFramework::SignalBase*, std::vector<SimFramework::Function*>> res = SimFramework::Internal::FunctionInputs(funcs);

    ASSERT_EQ(res.size(), 4);

    ASSERT_EQ(res[&s1].size(), 1);
    ASSERT_EQ(res[&s2].size(), 1);
    ASSERT_EQ(res[&s4].size(), 1);
    ASSERT_EQ(res[&s5].size(), 1);

    ASSERT_EQ(res[&s1][0], &a);
    ASSERT_EQ(res[&s2][0], &b);
    ASSERT_EQ(res[&s4][0], &c);
    ASSERT_EQ(res[&s5][0], &d);
}

TEST(FunctionInputs, SummingJunction) {
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

    std::map<SimFramework::SignalBase*, std::vector<SimFramework::Function*>> res = SimFramework::Internal::FunctionInputs(funcs);

    ASSERT_EQ(res.size(), 6);

    ASSERT_EQ(res[&s1].size(), 1);
    ASSERT_EQ(res[&s2].size(), 1);
    ASSERT_EQ(res[&s3].size(), 1);
    ASSERT_EQ(res[&s4].size(), 1);
    ASSERT_EQ(res[&s5].size(), 1);
    ASSERT_EQ(res[&s6].size(), 1);

    ASSERT_EQ(res[&s1][0], &a);
    ASSERT_EQ(res[&s2][0], &c);
    ASSERT_EQ(res[&s3][0], &b);
    ASSERT_EQ(res[&s4][0], &c);
    ASSERT_EQ(res[&s5][0], &c);
    ASSERT_EQ(res[&s6][0], &d);
}

TEST(FunctionInputs, Mask) {
    // Signals
    SimFramework::Signal<Eigen::Vector3f> s1;
    SimFramework::Signal<Eigen::Vector3f> s2;
    SimFramework::Signal<float> s3;
    SimFramework::Signal<float> s4;
    SimFramework::Signal<float> s5;
    SimFramework::Signal<float> s6;
    SimFramework::Signal<float> s7;
    SimFramework::Signal<float> s8;


    // Blocks
    SimFramework::Gain<Eigen::Vector3f, Eigen::Vector3f> a;
    SimFramework::Mask<Eigen::Vector3f, float> b;
    SimFramework::Gain<float, float> c;
    SimFramework::Gain<float, float> d;
    SimFramework::Gain<float, float> e;

    a.Configure(&s1, &s2, 1);
    b.Configure(&s2, {&s3, &s5, &s7}, {0, 1, 2});
    c.Configure(&s3, &s4, 1);
    d.Configure(&s5, &s6, 1);
    e.Configure(&s7, &s8, 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d, &e};

    std::map<SimFramework::SignalBase*, std::vector<SimFramework::Function*>> res = SimFramework::Internal::FunctionInputs(funcs);

    ASSERT_EQ(res.size(), 5);

    ASSERT_EQ(res[&s1].size(), 1);
    ASSERT_EQ(res[&s2].size(), 1);
    ASSERT_EQ(res[&s3].size(), 1);
    ASSERT_EQ(res[&s5].size(), 1);
    ASSERT_EQ(res[&s7].size(), 1);

    ASSERT_EQ(res[&s1][0], &a);
    ASSERT_EQ(res[&s2][0], &b);
    ASSERT_EQ(res[&s3][0], &c);
    ASSERT_EQ(res[&s5][0], &d);
    ASSERT_EQ(res[&s7][0], &e);
}

TEST(FunctionInputs, MultipleDependants) {
    // Signals
    SimFramework::Signal<Eigen::Vector2f> s1;
    SimFramework::Signal<Eigen::Vector2f> s2;
    SimFramework::Signal<Eigen::Vector2f> s3;
    SimFramework::Signal<Eigen::Vector2f> s4;
    SimFramework::Signal<Eigen::Vector2f> s5;
    SimFramework::Signal<float> s6;
    SimFramework::Signal<float> s7;
    SimFramework::Signal<float> s8;
    SimFramework::Signal<float> s9;

    // Blocks
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> a;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> b;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> c;
    SimFramework::SummingJunction<Eigen::Vector2f> d;
    SimFramework::Mask<Eigen::Vector2f, float> e;
    SimFramework::Gain<float, float> f;
    SimFramework::Gain<float, float> g;

    a.Configure(&s1, &s2, 1);
    b.Configure(&s2, &s3, 1);
    c.Configure(&s2, &s4, 1);
    d.Configure({&s3, &s4, &s2}, &s5, {1, 1, 1});
    e.Configure(&s5, {&s6, &s7}, {0, 1});
    f.Configure(&s6, &s8, 1);
    g.Configure(&s7, &s9, 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d, &e, &f, &g};

    std::map<SimFramework::SignalBase*, std::vector<SimFramework::Function*>> res = SimFramework::Internal::FunctionInputs(funcs);

    ASSERT_EQ(res.size(), 7);

    ASSERT_EQ(res[&s1].size(), 1);
    ASSERT_EQ(res[&s2].size(), 3);
    ASSERT_EQ(res[&s3].size(), 1);
    ASSERT_EQ(res[&s4].size(), 1);
    ASSERT_EQ(res[&s5].size(), 1);
    ASSERT_EQ(res[&s6].size(), 1);
    ASSERT_EQ(res[&s7].size(), 1);

    ASSERT_EQ(res[&s1][0], &a);

    // Assert all are different and one of the correct functions
    ASSERT_TRUE(res[&s2][0] != res[&s2][1] && res[&s2][0] != res[&s2][2] && res[&s2][1] != res[&s2][2]);
    ASSERT_TRUE(res[&s2][0] == &b || res[&s2][0] == &c || res[&s2][0] == &d);
    ASSERT_TRUE(res[&s2][1] == &b || res[&s2][1] == &c || res[&s2][1] == &d);
    ASSERT_TRUE(res[&s2][2] == &b || res[&s2][2] == &c || res[&s2][2] == &d);

    ASSERT_EQ(res[&s3][0], &d);
    ASSERT_EQ(res[&s4][0], &d);
    ASSERT_EQ(res[&s5][0], &e);
    ASSERT_EQ(res[&s6][0], &f);
    ASSERT_EQ(res[&s7][0], &g);
}


TEST(AdjacenyList, TwoParallelTrees) {
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

    std::vector<std::vector<SimFramework::Function*>> res = SimFramework::Internal::AdjacencyList(funcs);

    ASSERT_EQ(res.size(), 4);

    ASSERT_EQ(res[0].size(), 1);
    ASSERT_EQ(res[1].size(), 0);
    ASSERT_EQ(res[2].size(), 1);
    ASSERT_EQ(res[3].size(), 0);

    // Order should match input order
    ASSERT_EQ(res[0][0], &b);
    ASSERT_EQ(res[2][0], &d);
}

TEST(AdjacencyList, SummingJunction) {
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

    std::vector<std::vector<SimFramework::Function*>> res = SimFramework::Internal::AdjacencyList(funcs);

    ASSERT_EQ(res.size(), 4);

    ASSERT_EQ(res[0].size(), 1);
    ASSERT_EQ(res[1].size(), 1);
    ASSERT_EQ(res[2].size(), 1);
    ASSERT_EQ(res[3].size(), 0);

    // Order should match input order
    ASSERT_EQ(res[0][0], &c);
    ASSERT_EQ(res[1][0], &c);
    ASSERT_EQ(res[2][0], &d);
}

TEST(AdjacencyList, Mask) {
    // Signals
    SimFramework::Signal<Eigen::Vector3f> s1;
    SimFramework::Signal<Eigen::Vector3f> s2;
    SimFramework::Signal<float> s3;
    SimFramework::Signal<float> s4;
    SimFramework::Signal<float> s5;
    SimFramework::Signal<float> s6;
    SimFramework::Signal<float> s7;
    SimFramework::Signal<float> s8;


    // Blocks
    SimFramework::Gain<Eigen::Vector3f, Eigen::Vector3f> a;
    SimFramework::Mask<Eigen::Vector3f, float> b;
    SimFramework::Gain<float, float> c;
    SimFramework::Gain<float, float> d;
    SimFramework::Gain<float, float> e;

    a.Configure(&s1, &s2, 1);
    b.Configure(&s2, {&s3, &s5, &s7}, {0, 1, 2});
    c.Configure(&s3, &s4, 1);
    d.Configure(&s5, &s6, 1);
    e.Configure(&s7, &s8, 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d, &e};

    std::vector<std::vector<SimFramework::Function*>> res = SimFramework::Internal::AdjacencyList(funcs);

    ASSERT_EQ(res.size(), 5);

    ASSERT_EQ(res[0].size(), 1);
    ASSERT_EQ(res[1].size(), 3);
    ASSERT_EQ(res[2].size(), 0);
    ASSERT_EQ(res[3].size(), 0);
    ASSERT_EQ(res[4].size(), 0);

    // Order should match input order
    ASSERT_EQ(res[0][0], &b);
    ASSERT_TRUE(res[1][0] != res[1][1] && res[1][0] != res[1][2] && res[1][1] != res[1][2]);
    ASSERT_TRUE(res[1][0] == &c || res[1][0] == &d || res[1][0] == &e);
    ASSERT_TRUE(res[1][1] == &c || res[1][1] == &d || res[1][1] == &e);
    ASSERT_TRUE(res[1][2] == &c || res[1][2] == &d || res[1][2] == &e);
}

TEST(AdjacencyList, MultipleDependants) {
    // Signals
    SimFramework::Signal<Eigen::Vector2f> s1;
    SimFramework::Signal<Eigen::Vector2f> s2;
    SimFramework::Signal<Eigen::Vector2f> s3;
    SimFramework::Signal<Eigen::Vector2f> s4;
    SimFramework::Signal<Eigen::Vector2f> s5;
    SimFramework::Signal<float> s6;
    SimFramework::Signal<float> s7;
    SimFramework::Signal<float> s8;
    SimFramework::Signal<float> s9;

    // Blocks
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> a;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> b;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> c;
    SimFramework::SummingJunction<Eigen::Vector2f> d;
    SimFramework::Mask<Eigen::Vector2f, float> e;
    SimFramework::Gain<float, float> f;
    SimFramework::Gain<float, float> g;

    a.Configure(&s1, &s2, 1);
    b.Configure(&s2, &s3, 1);
    c.Configure(&s2, &s4, 1);
    d.Configure({&s3, &s4, &s2}, &s5, {1, 1, 1});
    e.Configure(&s5, {&s6, &s7}, {0, 1});
    f.Configure(&s6, &s8, 1);
    g.Configure(&s7, &s9, 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d, &e, &f, &g};

    std::vector<std::vector<SimFramework::Function*>> res = SimFramework::Internal::AdjacencyList(funcs);

    ASSERT_EQ(res.size(), 7);

    ASSERT_EQ(res[0].size(), 3);
    ASSERT_EQ(res[1].size(), 1);
    ASSERT_EQ(res[2].size(), 1);
    ASSERT_EQ(res[3].size(), 1);
    ASSERT_EQ(res[4].size(), 2);
    ASSERT_EQ(res[5].size(), 0);
    ASSERT_EQ(res[6].size(), 0);

    // Order should match input order
    ASSERT_TRUE(res[0][0] != res[0][1] && res[0][0] != res[0][2] && res[0][1] != res[0][2]);
    ASSERT_TRUE(res[0][0] == &b || res[0][0] == &c || res[0][0] == &d);
    ASSERT_TRUE(res[0][1] == &b || res[0][1] == &c || res[0][1] == &d);
    ASSERT_TRUE(res[0][2] == &b || res[0][2] == &c || res[0][2] == &d);
    ASSERT_EQ(res[1][0], &d);
    ASSERT_EQ(res[2][0], &d);
    ASSERT_EQ(res[3][0], &e);
    ASSERT_TRUE(res[4][0] != res[4][1]);
    ASSERT_TRUE(res[4][0] == &f || res[4][0] == &g);
    ASSERT_TRUE(res[4][1] == &f || res[4][1] == &g);
}

/*
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

TEST(FunctionOutputs, MultipleDependants) {
    // Signals
    SimFramework::Signal<Eigen::Vector2f> s1;
    SimFramework::Signal<Eigen::Vector2f> s2;
    SimFramework::Signal<Eigen::Vector2f> s3;
    SimFramework::Signal<Eigen::Vector2f> s4;
    SimFramework::Signal<Eigen::Vector2f> s5;
    SimFramework::Signal<float> s6;
    SimFramework::Signal<float> s7;
    SimFramework::Signal<float> s8;
    SimFramework::Signal<float> s9;

    // Blocks
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> a;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> b;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> c;
    SimFramework::SummingJunction<Eigen::Vector2f> d;
    SimFramework::Mask<Eigen::Vector2f, float> e;
    SimFramework::Gain<float, float> f;
    SimFramework::Gain<float, float> g;

    a.Configure(&s1, &s2, 1);
    b.Configure(&s2, &s3, 1);
    c.Configure(&s2, &s4, 1);
    d.Configure({&s3, &s4, &s2}, &s5, {1, 1, 1});
    e.Configure(&s5, {&s6, &s7}, {0, 1});
    f.Configure(&s6, &s8, 1);
    g.Configure(&s7, &s9, 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d, &e, &f, &g};

    std::map<SimFramework::SignalBase*, SimFramework::Function*> outputMap = SimFramework::Internal::FunctionOutputs(funcs);

    // Assertions
    ASSERT_EQ(outputMap.size(), 8);
    ASSERT_TRUE(outputMap[&s2] == &a);
    ASSERT_TRUE(outputMap[&s3] == &b);
    ASSERT_TRUE(outputMap[&s4] == &c);
    ASSERT_TRUE(outputMap[&s5] == &d);
    ASSERT_TRUE(outputMap[&s6] == &e);
    ASSERT_TRUE(outputMap[&s7] == &e);
    ASSERT_TRUE(outputMap[&s8] == &f);
    ASSERT_TRUE(outputMap[&s9] == &g);
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

TEST(UnpackTree, MultipleDependants) {
    // Blocks
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> a;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> b;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> c;
    SimFramework::SummingJunction<Eigen::Vector2f> d;
    SimFramework::Mask<Eigen::Vector2f, float> e;
    SimFramework::Gain<float, float> f;
    SimFramework::Gain<float, float> g;

    SimFramework::Internal::FunctionTree ta = {&a};
    SimFramework::Internal::FunctionTree tb = {&b};
    SimFramework::Internal::FunctionTree tc = {&c};
    SimFramework::Internal::FunctionTree td = {&d};
    SimFramework::Internal::FunctionTree te = {&e};
    SimFramework::Internal::FunctionTree tf = {&f};
    SimFramework::Internal::FunctionTree tg = {&g};


    tf.children.push_back(&te);
    tg.children.push_back(&te);
    te.children.push_back(&td);
    td.children.push_back(&tb);
    td.children.push_back(&tc);
    td.children.push_back(&ta);
    tb.children.push_back(&ta);
    tc.children.push_back(&ta);

    std::vector<SimFramework::Function*> o1 = SimFramework::Internal::UnpackTree(tf);
    std::vector<SimFramework::Function*> o2 = SimFramework::Internal::UnpackTree(tg);

    // Assertions

    // Should be a, b, c, d, e, f or a, c, b, d, e, f
    ASSERT_EQ(o1.size(), 6);

//    ASSERT_EQ(o1[0], &a);
//    ASSERT_EQ(o1[1], &a);
//    ASSERT_EQ(o1[2], &a);
//    ASSERT_TRUE(o1[3] == &b || o1[3] == &c);
//    ASSERT_TRUE(o1[4] == &b || o1[4] == &c);
    ASSERT_EQ(o1[5], &d);
    ASSERT_EQ(o1[6], &e);
    ASSERT_EQ(o1[7], &f);

    ASSERT_EQ(o2.size(), 8);
//    ASSERT_EQ(o2[0], &a);
//    ASSERT_EQ(o2[1], &a);
//    ASSERT_EQ(o2[2], &a);
//    ASSERT_TRUE(o2[3] == &b || o2[3] == &c);
//    ASSERT_TRUE(o2[4] == &b || o2[4] == &c);
    ASSERT_EQ(o2[5], &d);
    ASSERT_EQ(o2[6], &e);
    ASSERT_EQ(o2[7], &g);
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
//}*/