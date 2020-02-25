#include "Interpolation.h"

namespace SimFramework {


    InsertionResult1D Insertion1D(std::vector<float> &sortedArray, float element) {
        int low = 0;
        int high = sortedArray.size() - 1;

        int searchIndex;

        while (high - low != 1) {
            searchIndex = (low + high) / 2;

            if (sortedArray[searchIndex] > element) {
                high = searchIndex;
            } else {
                low = searchIndex;
            }
        }

        return {low, high};
    };



    float LinearInterp(float x, float x1, float x2, float y1, float y2)
    {
        return y1 + ((x - x1) / (x2 - x1)) * (y2 - y1);
    };

    float BilinearInterp(float x, float y, coord2D x1y1, coord2D x1y2, coord2D x2y1, coord2D x2y2) {

        // Interpolate to find f(x1, y), f(x2, y)
        float f1 = LinearInterp(y, x1y1.y, x1y2.y, x1y1.z, x1y2.z);
        float f2 = LinearInterp(y, x2y1.y, x2y2.y, x2y1.z, x2y2.z);

        return LinearInterp(x, x1y1.x, x2y2.x, f1, f2);
    }

    float InterpTable2D(Table2D &table, float x, float y) {
        // Get indices
        InsertionResult1D xIndices = Insertion1D(table.x, x);
        InsertionResult1D yIndices = Insertion1D(table.y, y);

        coord2D x1y1 = {table.x[xIndices.x1], table.y[yIndices.x1], table.z[yIndices.x1][xIndices.x1]};
        coord2D x1y2 = {table.x[xIndices.x1], table.y[yIndices.x2], table.z[yIndices.x2][xIndices.x1]};
        coord2D x2y1 = {table.x[xIndices.x2], table.y[yIndices.x1], table.z[yIndices.x1][xIndices.x2]};
        coord2D x2y2 = {table.x[xIndices.x2], table.y[yIndices.x2], table.z[yIndices.x2][xIndices.x2]};

        return BilinearInterp(x, y, x1y1, x1y2, x2y1, x2y2);
    }

} // namespace Framework