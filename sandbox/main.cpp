#include <iostream>
#include <string>
#include <vector>

#include "Framework.h"
#include "Components.h"

#include "Mass1D.h"
#include "SpringDamper1D.h"
#include "OutputBlock.h"


struct InsertionResult1D
{
    int x1;
    int x2;
};

InsertionResult1D Insertion1D(std::vector<float>& sortedArray, float element)
{
    int low = 0;
    int high = sortedArray.size() - 1;

    int searchIndex;

    while (high - low != 1)
    {
        searchIndex = (low + high) / 2;

        if (sortedArray[searchIndex] > element)
        {
            high = searchIndex;
        }

        else
        {
            low = searchIndex;
        }
    }

    return {low, high};
};


int main() {

    std::vector<float> array = {-10.5, -3.5, 0.5, 2.0};

    InsertionResult1D res = Insertion1D(array, -0.2);

    std::cout << "X1: " << res.x1 << ", X2: " << res.x2 << std::endl;

//    SimFramework::Signal<Eigen::Vector2f> signal1; // ConstantBlock States
//    SimFramework::Signal<float> signal2; // SpringDamper Force
//    SimFramework::Signal<float> signal3; // Const Additional Force
//    SimFramework::Signal<float> signal4; // Sum of forces
//    SimFramework::Signal<Eigen::Vector2f> signal5; // Mass1D States
//
//    // Create blocks
//    SimFramework::ConstantBlock<Eigen::Vector2f> connection1 (&signal1, {0.f, 0.f});
//    SimFramework::ConstantBlock<float> constForce (&signal3, -10.f);
//    SpringDamper1D spring(&signal1, &signal5, &signal2);
//    SimFramework::SummingJunction<float> sum({&signal2, &signal3}, &signal4, {1.f, 1.f});
//    Mass1D mass (&signal4, &signal5, {0.1, 0.f});
//    OutputBlock out(&signal5, &signal4);
//
//    // Construct system
//    SimFramework::SystemManager& systemManager = SimFramework::SystemManager::Get();
//    systemManager.RegisterBlocks({&connection1, &constForce}, {&mass}, {&spring, &sum}, {&out});
//    systemManager.Initialise(0.f);
//
//    // Iterate
//    for (float t = 0; t <= 1000; t += 0.01) {
//        systemManager.UpdateSystem(t);
//    }

    return 0;
}
