#include <vector>
#include <map>

#include "gtest/gtest.h"

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"


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

TEST(AssembleTree, SummingJunction) {
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
    std::vector<SimFramework::Internal::FunctionTree> ft = SimFramework::Internal::AssembleTree(funcs);

    // Assertions
    ASSERT_EQ(ft.size(), 1);
    ASSERT_TRUE(ft[0].block == &d);
    ASSERT_TRUE(ft[0].children[0]->block == &c);
    ASSERT_EQ(ft[0].children[0]->children.size(), 2);
}