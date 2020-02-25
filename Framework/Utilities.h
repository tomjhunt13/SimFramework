#ifndef SIMINTERFACE_UTILITIES_H
#define SIMINTERFACE_UTILITIES_H

#include <vector>
#include <cmath>

namespace SimFramework {

    float RadiansPerSecondToRPM(float radiansPerSecond);
    float RPMToRadiansPerSecond(float RPM);

    std::vector<float> TimeSteps(float tMin, float tMax, float dt);

} // namespace Framework

#endif //SIMINTERFACE_UTILITIES_H
