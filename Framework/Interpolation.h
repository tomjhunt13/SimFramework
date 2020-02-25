#ifndef SIMFRAMEWORK_INTERPOLATION_H
#define SIMFRAMEWORK_INTERPOLATION_H

#include <vector>

namespace SimFramework {

    struct Table3D {
        std::vector<float> x;
        std::vector<float> y;
        std::vector<std::vector<float>> z;
    };

    struct Coord2D
    {
        float x, y;
    };

    struct Coord3D
    {
        float x, y, z;
    };

    /**
     * Linear interpolation
     *
     * @param x Interpolation point
     * @param P1 Lower coordinate
     * @param P2 Upper coordinate
     *
     * @return Interpolated value at x
     */
    float LinearInterp(float x, Coord2D P1, Coord2D P2);

    /**
     * Bilinear interpolation
     *
     * @param x Interpolation point
     * @param P11 Coordinate at min x and min y
     * @param P21 Coordinate at min x and max y
     * @param P12 Coordinate at max x and min y
     * @param P22 Coordinate at max x and max y
     *
     * @return Interpolated value at x
     */
    float BilinearInterp(Coord2D x, Coord3D P11, Coord3D P21, Coord3D P12, Coord3D P22);

    /**
     * Piecewise linear interpolation of Table3D object
     *
     * @param table Reference to Table3D object
     * @param x Interpolation location
     *
     * @return Interpolation result
     */
    float InterpTable3D(Table3D& table, Coord2D x);

    namespace Internal
    {
        /**
         * Given a sorted array of floats, find two neighbouring elements E1 and E2 such that E1 <= x and E2 >= x and
         *  return corresponding element position indices
         *
         * if x < min(array), return 0 and 1
         * if x > max(array), return n and n-1
         *
         * @param sortedArray Array to search. Assumed length >= 2
         * @param x Value to find nearest indices of
         *
         * @return Two element array of neighbouring indices
         */
        std::vector<int> NearestIndices(std::vector<float> &sortedArray, float x);

    } // namespace Internal

} // namespace SimFramework

#endif //SIMFRAMEWORK_INTERPOLATION_H
