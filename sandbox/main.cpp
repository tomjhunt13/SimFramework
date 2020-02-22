#include <iostream>
#include <string>
#include <vector>

#include "Framework.h"
#include "Components.h"

#include "Mass1D.h"
#include "SpringDamper1D.h"
#include "OutputBlock.h"


struct Table2D
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<std::vector<float>> z;
};

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


struct coord2D{
    float x, y, z;
};


float LinearInterp(float x, float x1, float x2, float y1, float y2)
{
    return y1 + ((x - x1) / (x2 - x1)) * (y2 - y1);
}

float BilinearInterp(float x, float y, coord2D x1y1, coord2D x1y2, coord2D x2y1, coord2D x2y2)
{

    // Interpolate to find f(x1, y), f(x2, y)
    float f1 = LinearInterp(y, x1y1.y, x1y2.y, x1y1.z, x1y2.z);
    float f2 = LinearInterp(y, x2y1.y, x2y2.y, x2y1.z, x2y2.z);

    return LinearInterp(x, x1y1.x, x2y2.x, f1, f2);
}


float InterpTable2D(Table2D& table, float x, float y)
{
    // Get indices
    InsertionResult1D xIndices = Insertion1D(table.x, x);
    InsertionResult1D yIndices = Insertion1D(table.y, y);

    coord2D x1y1 = {table.x[xIndices.x1], table.y[yIndices.x1], table.z[yIndices.x1][xIndices.x1]};
    coord2D x1y2 = {table.x[xIndices.x1], table.y[yIndices.x2], table.z[yIndices.x2][xIndices.x1]};
    coord2D x2y1 = {table.x[xIndices.x2], table.y[yIndices.x1], table.z[yIndices.x1][xIndices.x2]};
    coord2D x2y2 = {table.x[xIndices.x2], table.y[yIndices.x2], table.z[yIndices.x2][xIndices.x2]};

    return BilinearInterp(x, y, x1y1, x1y2, x2y1, x2y2);
}


int main() {

    Table2D tab;
    tab.x =  {1.5, 2.5, 4.f};
    tab.y = {0.5, 1.2, 2.2};
    tab.z = {{6.f, 5.f, 1.f}, {1.f, 4.f, 5.f}, {2.f, 3.f, 2.f}};

    std::cout << InterpTable2D(tab, 1.5, 2.1) << std::endl;


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
