#ifndef SIMFRAMEWORK_INTERPOLATION_H
#define SIMFRAMEWORK_INTERPOLATION_H

#include <vector>

namespace SimFramework {

    struct Table2D {
        std::vector<float> x;
        std::vector<float> y;
        std::vector<std::vector<float>> z;
    };

    struct InsertionResult1D {
        int x1;
        int x2;
    };

    struct coord2D {
        float x, y, z;
    };

    InsertionResult1D Insertion1D(std::vector<float> &sortedArray, float element);


    float LinearInterp(float x, float x1, float x2, float y1, float y2);

    float BilinearInterp(float x, float y, coord2D x1y1, coord2D x1y2, coord2D x2y1, coord2D x2y2);


    float InterpTable2D(Table2D &table, float x, float y);

} // namespace Framework

#endif //SIMFRAMEWORK_INTERPOLATION_H
