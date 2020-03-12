#include <vector>
#include <map>

#include "gtest/gtest.h"

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"


TEST(AssembleTree, test1) {
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


    SimFramework::Function* pb  = &b;
    SimFramework::Function* pd  = &d;



    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d};
    std::vector<SimFramework::Internal::FunctionTree> ft = SimFramework::Internal::AssembleTree(funcs);

    // Assertions
    ASSERT_EQ(ft.size(), 2);

    ASSERT_TRUE(ft[0].block == &b || ft[0].block == &d);
    ASSERT_TRUE(ft[1].block == &b || ft[1].block == &d);

    int kja = 1;
}