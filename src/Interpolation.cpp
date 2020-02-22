#include "Interpolation.h"


#include <vector>
#include "Eigen/Dense"

struct Table2D
{
    std::vector<float> x = {1.5, 2.5, 4.f};
    std::vector<float> y = {0.5, 1.2, 2.2};
    std::vector<std::vector<float>> z = {{6.f, 5.f, 1.f}, {1.f, 4.f, 5.f}, {2.f, 3.f, 2.f}};
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

float LinearInterp2D(float x, float y, float x1, float x2, float y1, float y2)
{
    return ((x - x1) / (x2 - x1)) * (y2 - y1) + y1;
}


float InterpTable2D(Table2D& table, float x, float y)
{
    // Get indices
    InsertionResult1D xIndices = Insertion1D(table.x, x);
    InsertionResult1D yIndices = Insertion1D(table.y, y);

//    return LinearInterp2D(x, y, )
}

