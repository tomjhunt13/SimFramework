#include <vector>
#include <map>

#include "gtest/gtest.h"
#include "Eigen/Dense"

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

template <typename Type>
bool EqualVectors(std::vector<Type> vector1, std::vector<Type> vector2)
{
    if (vector1.size() != vector2.size())
    {
        return false;
    }

    for (int i = 0; i < vector1.size(); i++)
    {
        if (vector1[i] != vector2[i])
        {
            return false;
        }
    }

    return true;
}

TEST(FunctionInputs, TwoParallelTrees) {
    // Signals
    SimFramework::Signal<float> s1;
//    SimFramework::Signal<float> s2;
//    SimFramework::Signal<float> s3;
    SimFramework::Signal<float> s4;
//    SimFramework::Signal<float> s5;
//    SimFramework::Signal<float> s6;

    // Blocks
    SimFramework::Gain<float, float> a;
    SimFramework::Gain<float, float> b;
    SimFramework::Gain<float, float> c;
    SimFramework::Gain<float, float> d;

    a.Configure(&s1, 1);
    b.Configure(a.OutSignal(), 1);
    c.Configure(&s4, 1);
    d.Configure(c.OutSignal(), 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d};

    std::map<const SimFramework::SignalBase*, std::vector<SimFramework::Function*>> res = SimFramework::Internal::FunctionInputs(funcs);

    ASSERT_EQ(res.size(), 4);

    ASSERT_EQ(res[&s1].size(), 1);
    ASSERT_EQ(res[a.OutSignal()].size(), 1);
    ASSERT_EQ(res[&s4].size(), 1);
    ASSERT_EQ(res[c.OutSignal()].size(), 1);

    ASSERT_EQ(res[&s1][0], &a);
    ASSERT_EQ(res[a.OutSignal()][0], &b);
    ASSERT_EQ(res[&s4][0], &c);
    ASSERT_EQ(res[c.OutSignal()][0], &d);
}

TEST(FunctionInputs, SummingJunction) {
    // Signals
    SimFramework::Signal<float> s1;
//    SimFramework::Signal<float> s2;
    SimFramework::Signal<float> s3;
//    SimFramework::Signal<float> s4;
    SimFramework::Signal<float> s5;
//    SimFramework::Signal<float> s6;
//    SimFramework::Signal<float> s7;

    // Blocks
    SimFramework::Gain<float, float> a;
    SimFramework::Gain<float, float> b;
    SimFramework::SummingJunction<float> c;
    SimFramework::Gain<float, float> d;

    a.Configure(&s1, 1);
    b.Configure(&s3, 1);
    c.Configure({a.OutSignal(), b.OutSignal(), &s5}, {1.f, 1.f, 1.f});
    d.Configure(c.OutSignal(), 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d};

    std::map<const SimFramework::SignalBase*, std::vector<SimFramework::Function*>> res = SimFramework::Internal::FunctionInputs(funcs);

    ASSERT_EQ(res.size(), 6);

    ASSERT_EQ(res[&s1].size(), 1);
    ASSERT_EQ(res[a.OutSignal()].size(), 1);
    ASSERT_EQ(res[&s3].size(), 1);
    ASSERT_EQ(res[b.OutSignal()].size(), 1);
    ASSERT_EQ(res[&s5].size(), 1);
    ASSERT_EQ(res[c.OutSignal()].size(), 1);

    ASSERT_EQ(res[&s1][0], &a);
    ASSERT_EQ(res[a.OutSignal()][0], &c);
    ASSERT_EQ(res[&s3][0], &b);
    ASSERT_EQ(res[b.OutSignal()][0], &c);
    ASSERT_EQ(res[&s5][0], &c);
    ASSERT_EQ(res[c.OutSignal()][0], &d);
}

TEST(FunctionInputs, Mask) {
    // Signals
    SimFramework::Signal<Eigen::Vector3f> s1;
//    SimFramework::Signal<Eigen::Vector3f> s2;
//    SimFramework::Signal<float> s3;
    SimFramework::Signal<float> s4;
//    SimFramework::Signal<float> s5;
    SimFramework::Signal<float> s6;
//    SimFramework::Signal<float> s7;
    SimFramework::Signal<float> s8;


    // Blocks
    SimFramework::Gain<Eigen::Vector3f, Eigen::Vector3f> a;
    SimFramework::Mask<Eigen::Vector3f, float, 3> b;
    SimFramework::Gain<float, float> c;
    SimFramework::Gain<float, float> d;
    SimFramework::Gain<float, float> e;

    a.Configure(&s1, 1);
    b.Configure(a.OutSignal());
    c.Configure(b.OutSignal(0), 1);
    d.Configure(b.OutSignal(1), 1);
    e.Configure(b.OutSignal(2), 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d, &e};

    std::map<const SimFramework::SignalBase*, std::vector<SimFramework::Function*>> res = SimFramework::Internal::FunctionInputs(funcs);

    ASSERT_EQ(res.size(), 5);

    ASSERT_EQ(res[&s1].size(), 1);
    ASSERT_EQ(res[a.OutSignal()].size(), 1);
    ASSERT_EQ(res[b.OutSignal(0)].size(), 1);
    ASSERT_EQ(res[b.OutSignal(1)].size(), 1);
    ASSERT_EQ(res[b.OutSignal(2)].size(), 1);

    ASSERT_EQ(res[&s1][0], &a);
    ASSERT_EQ(res[a.OutSignal()][0], &b);
    ASSERT_EQ(res[b.OutSignal(0)][0], &c);
    ASSERT_EQ(res[b.OutSignal(1)][0], &d);
    ASSERT_EQ(res[b.OutSignal(2)][0], &e);
}

TEST(FunctionInputs, MultipleDependants) {
    // Signals
    SimFramework::Signal<Eigen::Vector2f> s1;
//    SimFramework::Signal<Eigen::Vector2f> s2;
//    SimFramework::Signal<Eigen::Vector2f> s3;
//    SimFramework::Signal<Eigen::Vector2f> s4;
//    SimFramework::Signal<Eigen::Vector2f> s5;
//    SimFramework::Signal<float> s6;
//    SimFramework::Signal<float> s7;
//    SimFramework::Signal<float> s8;
//    SimFramework::Signal<float> s9;

    // Blocks
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> a;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> b;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> c;
    SimFramework::SummingJunction<Eigen::Vector2f> d;
    SimFramework::Mask<Eigen::Vector2f, float, 2> e;
    SimFramework::Gain<float, float> f;
    SimFramework::Gain<float, float> g;

    a.Configure(&s1, 1);
    b.Configure(a.OutSignal(), 1);
    c.Configure(a.OutSignal(), 1);
    d.Configure({b.OutSignal(), c.OutSignal(), a.OutSignal()}, {1, 1, 1});
    e.Configure(d.OutSignal());
    f.Configure(e.OutSignal(0), 1);
    g.Configure(e.OutSignal(1), 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d, &e, &f, &g};

    std::map<const SimFramework::SignalBase*, std::vector<SimFramework::Function*>> res = SimFramework::Internal::FunctionInputs(funcs);

    ASSERT_EQ(res.size(), 7);

    ASSERT_EQ(res[&s1].size(), 1);
    ASSERT_EQ(res[a.OutSignal()].size(), 3);
    ASSERT_EQ(res[b.OutSignal()].size(), 1);
    ASSERT_EQ(res[c.OutSignal()].size(), 1);
    ASSERT_EQ(res[d.OutSignal()].size(), 1);
    ASSERT_EQ(res[e.OutSignal(0)].size(), 1);
    ASSERT_EQ(res[e.OutSignal(1)].size(), 1);

    ASSERT_EQ(res[&s1][0], &a);

    // Assert all are different and one of the correct functions
    ASSERT_TRUE(res[a.OutSignal()][0] != res[a.OutSignal()][1] && res[a.OutSignal()][0] != res[a.OutSignal()][2] && res[a.OutSignal()][1] != res[a.OutSignal()][2]);
    ASSERT_TRUE(res[a.OutSignal()][0] == &b || res[a.OutSignal()][0] == &c || res[a.OutSignal()][0] == &d);
    ASSERT_TRUE(res[a.OutSignal()][1] == &b || res[a.OutSignal()][1] == &c || res[a.OutSignal()][1] == &d);
    ASSERT_TRUE(res[a.OutSignal()][2] == &b || res[a.OutSignal()][2] == &c || res[a.OutSignal()][2] == &d);

    ASSERT_EQ(res[b.OutSignal()][0], &d);
    ASSERT_EQ(res[c.OutSignal()][0], &d);
    ASSERT_EQ(res[d.OutSignal()][0], &e);
    ASSERT_EQ(res[e.OutSignal(0)][0], &f);
    ASSERT_EQ(res[e.OutSignal(1)][0], &g);
}


TEST(AdjacenyList, TwoParallelTrees) {
    // Signals
    SimFramework::Signal<float> s1;
//    SimFramework::Signal<float> s2;
//    SimFramework::Signal<float> s3;
    SimFramework::Signal<float> s4;
//    SimFramework::Signal<float> s5;
//    SimFramework::Signal<float> s6;

    // Blocks
    SimFramework::Gain<float, float> a;
    SimFramework::Gain<float, float> b;
    SimFramework::Gain<float, float> c;
    SimFramework::Gain<float, float> d;

    a.Configure(&s1, 1);
    b.Configure(a.OutSignal(), 1);
    c.Configure(&s4, 1);
    d.Configure(c.OutSignal(), 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d};

    std::vector<std::vector<int>> res = SimFramework::Internal::AdjacencyList(funcs);

    ASSERT_EQ(res.size(), 4);

    ASSERT_EQ(res[0].size(), 1);
    ASSERT_EQ(res[1].size(), 0);
    ASSERT_EQ(res[2].size(), 1);
    ASSERT_EQ(res[3].size(), 0);

    // Order should match input order
    ASSERT_EQ(res[0][0], 1);
    ASSERT_EQ(res[2][0], 3);
}

TEST(AdjacencyList, SummingJunction) {
    // Signals
    SimFramework::Signal<float> s1;
//    SimFramework::Signal<float> s2;
    SimFramework::Signal<float> s3;
//    SimFramework::Signal<float> s4;
    SimFramework::Signal<float> s5;
//    SimFramework::Signal<float> s6;
//    SimFramework::Signal<float> s7;

    // Blocks
    SimFramework::Gain<float, float> a;
    SimFramework::Gain<float, float> b;
    SimFramework::SummingJunction<float> c;
    SimFramework::Gain<float, float> d;

    a.Configure(&s1, 1);
    b.Configure(&s3, 1);
    c.Configure({a.OutSignal(), b.OutSignal(), &s5}, {1.f, 1.f, 1.f});
    d.Configure(c.OutSignal(), 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d};

    std::vector<std::vector<int>> res = SimFramework::Internal::AdjacencyList(funcs);

    ASSERT_EQ(res.size(), 4);

    ASSERT_EQ(res[0].size(), 1);
    ASSERT_EQ(res[1].size(), 1);
    ASSERT_EQ(res[2].size(), 1);
    ASSERT_EQ(res[3].size(), 0);

    // Order should match input order
    ASSERT_EQ(res[0][0], 2);
    ASSERT_EQ(res[1][0], 2);
    ASSERT_EQ(res[2][0], 3);
}

TEST(AdjacencyList, Mask) {
    // Signals
    SimFramework::Signal<Eigen::Vector3f> s1;
//    SimFramework::Signal<Eigen::Vector3f> s2;
//    SimFramework::Signal<float> s3;
    SimFramework::Signal<float> s4;
//    SimFramework::Signal<float> s5;
    SimFramework::Signal<float> s6;
//    SimFramework::Signal<float> s7;
    SimFramework::Signal<float> s8;


    // Blocks
    SimFramework::Gain<Eigen::Vector3f, Eigen::Vector3f> a;
    SimFramework::Mask<Eigen::Vector3f, float, 3> b;
    SimFramework::Gain<float, float> c;
    SimFramework::Gain<float, float> d;
    SimFramework::Gain<float, float> e;

    a.Configure(&s1, 1);
    b.Configure(a.OutSignal());
    c.Configure(b.OutSignal(0), 1);
    d.Configure(b.OutSignal(1), 1);
    e.Configure(b.OutSignal(2), 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d, &e};

    std::vector<std::vector<int>> res = SimFramework::Internal::AdjacencyList(funcs);

    ASSERT_EQ(res.size(), 5);

    ASSERT_EQ(res[0].size(), 1);
    ASSERT_EQ(res[1].size(), 3);
    ASSERT_EQ(res[2].size(), 0);
    ASSERT_EQ(res[3].size(), 0);
    ASSERT_EQ(res[4].size(), 0);

    // Order should match input order
    ASSERT_EQ(res[0][0], 1);
    ASSERT_TRUE(res[1][0] != res[1][1] && res[1][0] != res[1][2] && res[1][1] != res[1][2]);
    ASSERT_TRUE(res[1][0] == 2 || res[1][0] == 3 || res[1][0] == 4);
    ASSERT_TRUE(res[1][1] == 2 || res[1][1] == 3 || res[1][1] == 4);
    ASSERT_TRUE(res[1][2] == 2 || res[1][2] == 3 || res[1][2] == 4);
}

TEST(AdjacencyList, MultipleDependants) {
    // Signals
    SimFramework::Signal<Eigen::Vector2f> s1;
//    SimFramework::Signal<Eigen::Vector2f> s2;
//    SimFramework::Signal<Eigen::Vector2f> s3;
//    SimFramework::Signal<Eigen::Vector2f> s4;
//    SimFramework::Signal<Eigen::Vector2f> s5;
//    SimFramework::Signal<float> s6;
//    SimFramework::Signal<float> s7;
//    SimFramework::Signal<float> s8;
//    SimFramework::Signal<float> s9;

    // Blocks
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> a;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> b;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> c;
    SimFramework::SummingJunction<Eigen::Vector2f> d;
    SimFramework::Mask<Eigen::Vector2f, float, 2> e;
    SimFramework::Gain<float, float> f;
    SimFramework::Gain<float, float> g;

    a.Configure(&s1, 1);
    b.Configure(a.OutSignal(), 1);
    c.Configure(a.OutSignal(), 1);
    d.Configure({b.OutSignal(), c.OutSignal(), a.OutSignal()}, {1, 1, 1});
    e.Configure(d.OutSignal());
    f.Configure(e.OutSignal(0), 1);
    g.Configure(e.OutSignal(1), 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d, &e, &f, &g};

    std::vector<std::vector<int>> res = SimFramework::Internal::AdjacencyList(funcs);

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
    ASSERT_TRUE(res[0][0] == 1 || res[0][0] == 2 || res[0][0] == 3);
    ASSERT_TRUE(res[0][1] == 1 || res[0][1] == 2 || res[0][1] == 3);
    ASSERT_TRUE(res[0][2] == 1 || res[0][2] == 2 || res[0][2] == 3);
    ASSERT_EQ(res[1][0], 3);
    ASSERT_EQ(res[2][0], 3);
    ASSERT_EQ(res[3][0], 4);
    ASSERT_TRUE(res[4][0] != res[4][1]);
    ASSERT_TRUE(res[4][0] == 5 || res[4][0] == 6);
    ASSERT_TRUE(res[4][1] == 5 || res[4][1] == 6);
}


TEST(TopologicalSort, TwoParallelTrees) {

    // Problem
    std::vector<std::vector<int>> adjacencyList = {{1}, {}, {3}, {}};
    std::vector<int> res = SimFramework::Internal::TopologicalSort(adjacencyList);

    // Possible solutions
    std::vector<std::vector<int>> solutions = {{0, 1, 2, 3}, {2, 3, 0, 1}};

    ASSERT_TRUE(EqualVectors<int>(res, solutions[0]) || EqualVectors<int>(res, solutions[1]));
}

TEST(TopologicalSort, SummingJunction) {

    // Problem
    std::vector<std::vector<int>> adjacencyList = {{2}, {2}, {3}, {}};
    std::vector<int> res = SimFramework::Internal::TopologicalSort(adjacencyList);

    // Possible solutions
    std::vector<std::vector<int>> solutions = {{0, 1, 2, 3}, {1, 0, 2, 3}};

    ASSERT_TRUE(EqualVectors<int>(res, solutions[0]) || EqualVectors<int>(res, solutions[1]));
}

TEST(TopologicalSort, Mask) {

    // Problem
    std::vector<std::vector<int>> adjacencyList = {{1}, {2, 3, 4}, {}, {}, {}};
    std::vector<int> res = SimFramework::Internal::TopologicalSort(adjacencyList);

    // Possible solutions
    std::vector<std::vector<int>> solutions = {
            {0, 1, 2, 3, 4},
            {0, 1, 2, 4, 3},
            {0, 1, 3, 2, 4},
            {0, 1, 3, 4, 2},
            {0, 1, 4, 2, 3},
            {0, 1, 4, 3, 2}};

    bool test = (EqualVectors<int>(res, solutions[0]) ||
                 EqualVectors<int>(res, solutions[1]) ||
                 EqualVectors<int>(res, solutions[2]) ||
                 EqualVectors<int>(res, solutions[3]) ||
                 EqualVectors<int>(res, solutions[4]) ||
                 EqualVectors<int>(res, solutions[5]));

    ASSERT_TRUE(test);
}

TEST(TopologicalSort, MultipleDependants) {

    // Problem
    std::vector<std::vector<int>> adjacencyList = {{1, 2}, {3}, {3}, {4},  {5, 6}, {}, {}};
    std::vector<int> res = SimFramework::Internal::TopologicalSort(adjacencyList);

    // Possible solutions
    std::vector<std::vector<int>> solutions = {
            {0, 1, 2, 3, 4, 5, 6},
            {0, 1, 2, 3, 4, 6, 5},
            {0, 2, 1, 3, 4, 5, 6},
            {0, 2, 1, 3, 4, 6, 5}};

    bool test = (EqualVectors<int>(res, solutions[0]) ||
                 EqualVectors<int>(res, solutions[1]) ||
                 EqualVectors<int>(res, solutions[2]) ||
                 EqualVectors<int>(res, solutions[3]));

    ASSERT_TRUE(test);
}

TEST(TopologicalSort, ExtraTest) {

    // Problem
    std::vector<std::vector<int>> adjacencyList = {{1}, {2, 4, 7}, {3}, {5}, {6}, {8}, {8}, {8}, {9, 10}, {}, {}};
    std::vector<int> res = SimFramework::Internal::TopologicalSort(adjacencyList);

    // Possible solutions
    std::vector<std::vector<int>> solutions = {
            {0, 1, 2, 3, 5, 4, 6, 7, 8, 9, 10},
            {0, 1, 2, 3, 5, 4, 6, 7, 8, 10, 9},

            {0, 1, 2, 3, 5, 7, 4, 6, 8, 9, 10},
            {0, 1, 2, 3, 5, 7, 4, 6, 8, 10, 9},

            {0, 1, 4, 6, 2, 3, 5, 7, 8, 9, 10},
            {0, 1, 4, 6, 2, 3, 5, 7, 8, 10, 9},

            {0, 1, 4, 6, 7, 2, 3, 5, 8, 9, 10},
            {0, 1, 4, 6, 7, 2, 3, 5, 8, 10, 9},

            {0, 1, 7, 4, 6, 2, 3, 5, 8, 9, 10},
            {0, 1, 7, 4, 6, 2, 3, 5, 8, 10, 9},

            {0, 1, 7, 2, 3, 5, 4, 6, 8, 9, 10},
            {0, 1, 7, 2, 3, 4, 4, 6, 8, 10, 9}};


    bool test = (EqualVectors<int>(res, solutions[0]) ||
                 EqualVectors<int>(res, solutions[1]) ||
                 EqualVectors<int>(res, solutions[2]) ||
                 EqualVectors<int>(res, solutions[3]) ||
                 EqualVectors<int>(res, solutions[4]) ||
                 EqualVectors<int>(res, solutions[5]) ||
                 EqualVectors<int>(res, solutions[6]) ||
                 EqualVectors<int>(res, solutions[7]) ||
                 EqualVectors<int>(res, solutions[8]) ||
                 EqualVectors<int>(res, solutions[9]) ||
                 EqualVectors<int>(res, solutions[10]) ||
                 EqualVectors<int>(res, solutions[11]));

    ASSERT_TRUE(test);

}


TEST(SortFunctions, TwoParallelTrees) {
    // Signals
    SimFramework::Signal<float> s1;
//    SimFramework::Signal<float> s2;
//    SimFramework::Signal<float> s3;
    SimFramework::Signal<float> s4;
//    SimFramework::Signal<float> s5;
//    SimFramework::Signal<float> s6;

    // Blocks
    SimFramework::Gain<float, float> a;
    SimFramework::Gain<float, float> b;
    SimFramework::Gain<float, float> c;
    SimFramework::Gain<float, float> d;

    a.Configure(&s1, 1);
    b.Configure(a.OutSignal(), 1);
    c.Configure(&s4, 1);
    d.Configure(c.OutSignal(), 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d};
    std::vector<SimFramework::Function*> sortedFuncs = SimFramework::Internal::SortFunctions(funcs);

    // Possible solutions
    std::vector<std::vector<SimFramework::Function*>> solutions = {{&a, &b, &c, &d}, {&c, &d, &a, &b}};
    ASSERT_TRUE(EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[0]) || EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[1]));

}

TEST(SortFunctions, SummingJunction) {
    // Signals
    SimFramework::Signal<float> s1;
//    SimFramework::Signal<float> s2;
    SimFramework::Signal<float> s3;
//    SimFramework::Signal<float> s4;
    SimFramework::Signal<float> s5;
//    SimFramework::Signal<float> s6;
//    SimFramework::Signal<float> s7;

    // Blocks
    SimFramework::Gain<float, float> a;
    SimFramework::Gain<float, float> b;
    SimFramework::SummingJunction<float> c;
    SimFramework::Gain<float, float> d;

    a.Configure(&s1, 1);
    b.Configure(&s3, 1);
    c.Configure({a.OutSignal(), b.OutSignal(), &s5}, {1.f, 1.f, 1.f});
    d.Configure(c.OutSignal(), 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d};
    std::vector<SimFramework::Function*> sortedFuncs = SimFramework::Internal::SortFunctions(funcs);

    // Possible solutions
    std::vector<std::vector<SimFramework::Function*>> solutions = {{&a, &b, &c, &d}, {&b, &a, &c, &d}};

    ASSERT_TRUE(EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[0]) || EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[1]));
}

TEST(SortFunctions, Mask) {
    // Signals
    SimFramework::Signal<Eigen::Vector3f> s1;
//    SimFramework::Signal<Eigen::Vector3f> s2;
//    SimFramework::Signal<float> s3;
    SimFramework::Signal<float> s4;
//    SimFramework::Signal<float> s5;
    SimFramework::Signal<float> s6;
//    SimFramework::Signal<float> s7;
    SimFramework::Signal<float> s8;


    // Blocks
    SimFramework::Gain<Eigen::Vector3f, Eigen::Vector3f> a;
    SimFramework::Mask<Eigen::Vector3f, float, 3> b;
    SimFramework::Gain<float, float> c;
    SimFramework::Gain<float, float> d;
    SimFramework::Gain<float, float> e;

    a.Configure(&s1, 1);
    b.Configure(a.OutSignal());
    c.Configure(b.OutSignal(0), 1);
    d.Configure(b.OutSignal(1), 1);
    e.Configure(b.OutSignal(2), 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d, &e};
    std::vector<SimFramework::Function*> sortedFuncs = SimFramework::Internal::SortFunctions(funcs);

    // Possible solutions
    std::vector<std::vector<SimFramework::Function*>> solutions = {
            {&a, &b, &c, &d, &e},
            {&a, &b, &c, &e, &d},
            {&a, &b, &d, &c, &e},
            {&a, &b, &d, &e, &c},
            {&a, &b, &e, &c, &d},
            {&a, &b, &e, &d, &c}};

    bool test = (EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[0]) ||
                 EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[1]) ||
                 EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[2]) ||
                 EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[3]) ||
                 EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[4]) ||
                 EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[5]));

    ASSERT_TRUE(test);
}

TEST(SortFunctions, MultipleDependants) {
    // Signals
    SimFramework::Signal<Eigen::Vector2f> s1;
//    SimFramework::Signal<Eigen::Vector2f> s2;
//    SimFramework::Signal<Eigen::Vector2f> s3;
//    SimFramework::Signal<Eigen::Vector2f> s4;
//    SimFramework::Signal<Eigen::Vector2f> s5;
//    SimFramework::Signal<float> s6;
//    SimFramework::Signal<float> s7;
//    SimFramework::Signal<float> s8;
//    SimFramework::Signal<float> s9;

    // Blocks
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> a;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> b;
    SimFramework::Gain<Eigen::Vector2f, Eigen::Vector2f> c;
    SimFramework::SummingJunction<Eigen::Vector2f> d;
    SimFramework::Mask<Eigen::Vector2f, float, 2> e;
    SimFramework::Gain<float, float> f;
    SimFramework::Gain<float, float> g;

    a.Configure(&s1, 1);
    b.Configure(a.OutSignal(), 1);
    c.Configure(a.OutSignal(), 1);
    d.Configure({b.OutSignal(), c.OutSignal(), a.OutSignal()}, {1, 1, 1});
    e.Configure(d.OutSignal());
    f.Configure(e.OutSignal(0), 1);
    g.Configure(e.OutSignal(1), 1);

    std::vector<SimFramework::Function*> funcs = {&a, &b, &c, &d, &e, &f, &g};
    std::vector<SimFramework::Function*> sortedFuncs = SimFramework::Internal::SortFunctions(funcs);

    // Possible solutions
    std::vector<std::vector<SimFramework::Function*>> solutions = {
            {&a, &b, &c, &d, &e, &f, &g},
            {&a, &b, &c, &d, &e, &g, &f},
            {&a, &c, &b, &d, &e, &f, &g},
            {&a, &c, &b, &d, &e, &g, &f}};

    bool test = (EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[0]) ||
                 EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[1]) ||
                 EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[2]) ||
                 EqualVectors<SimFramework::Function*>(sortedFuncs, solutions[3]));

    ASSERT_TRUE(test);
}


class PureFunctions : public SimFramework::System
{
public:
    PureFunctions() : System(1.f)
    {
        // Configure blocks
        this->In1.Configure(0.f);
        this->In2.Configure(0.f);
        this->G1.Configure(this->In1.OutSignal(), 2.f);
        this->G2.Configure(this->In2.OutSignal(), 3.f);
        this->Sum.Configure({this->G1.OutSignal(), this->G2.OutSignal()},  {1.f, -1.f});
        this->G3.Configure(this->Sum.OutSignal(), 4.f);
        this->Out.Configure(this->G3.OutSignal(), 0.f);

        SimFramework::BlockList list = {{&(this->In1), &(this->In2)},
                                        {},
                                        {&(this->G3), &(this->Sum), &(this->G2), &(this->G1)},
                                        {&(this->Out)}};
        this->RegisterBlocks(list);
    };

    void Input(float in1, float in2)
    {
        this->In1.WriteValue(in1);
        this->In2.WriteValue(in2);
    };

    float Output()
    {
        return this->Out.ReadValue();
    };

private:

    // Blocks
    SimFramework::Input<float> In1;
    SimFramework::Input<float> In2;
    SimFramework::Output<float> Out;
    SimFramework::Gain<float, float> G1;
    SimFramework::Gain<float, float> G2;
    SimFramework::Gain<float, float> G3;
    SimFramework::SummingJunction<float> Sum;
};

TEST(SortedModel, PureFunctions)
{
    PureFunctions p;

    p.Input(3.f, 4.f);
    p.Initialise(0.f);
    p.Update(1.f);
    ASSERT_FLOAT_EQ(p.Output(), -24.f);

    p.Input(5.f, -2.f);
    p.Initialise(0.f);
    p.Update(1.f);
    ASSERT_FLOAT_EQ(p.Output(), 64.f);
}