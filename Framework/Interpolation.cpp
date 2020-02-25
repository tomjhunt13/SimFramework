#include "Interpolation.h"

namespace SimFramework {


    std::vector<int> Internal::NearestIndices(std::vector<float> &sortedArray, float x) {
        // Implementation uses a modified binary search algorithm
        int low = 0;
        int high = sortedArray.size() - 1;
        int searchIndex;

        while (high - low != 1) {
            searchIndex = (low + high) / 2;

            if (sortedArray[searchIndex] > x) {
                high = searchIndex;
            } else {
                low = searchIndex;
            }
        }

        return {low, high};
    };



    float LinearInterp(float x, Coord2D P1, Coord2D P2)
    {
        return P1.y + ((x - P1.x) / (P2.x - P1.x)) * (P2.y - P1.y);
    };

    float BilinearInterp(Coord2D x, Coord3D P11, Coord3D P21, Coord3D P12, Coord3D P22)
    {

        // Interpolate in x direction: f1 - P11, P12, f2 - P2, P22
        float f1 = LinearInterp(x.x, {P11.x, P11.z}, {P12.x, P12.z});
        float f2 = LinearInterp(x.x, {P21.x, P21.z}, {P22.x, P22.z});

        // Interpolate results in y direction
        return LinearInterp(x.y, {P11.y, f1}, {P21.y, f2});
    }

    float InterpTable3D(Table3D& table, Coord2D x) {
        // Get indices
        std::vector<int> i = Internal::NearestIndices(table.x, x.x);
        std::vector<int> j = Internal::NearestIndices(table.y, x.y);

        Coord3D P11 = {table.x[i[0]], table.y[j[0]], table.z[j[0]][i[0]]};
        Coord3D P12 = {table.x[i[1]], table.y[j[0]], table.z[j[0]][i[1]]};
        Coord3D P21 = {table.x[i[0]], table.y[j[1]], table.z[j[1]][i[0]]};
        Coord3D P22 = {table.x[i[1]], table.y[j[1]], table.z[j[1]][i[1]]};

        return BilinearInterp(x, P11, P21, P12, P22);
    }

} // namespace SimFramework