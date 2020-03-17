#ifndef SIMINTERFACE_UTILITIES_H
#define SIMINTERFACE_UTILITIES_H

#include <vector>
#include <string>

#include "Interpolation.h"

namespace SimFramework {


    // Unit conversions
    float RadiansPerSecondToRPM(float radiansPerSecond);
    float RPMToRadiansPerSecond(float RPM);
    const float pi();

    Table3D ReadTableJSON(std::string JSONFilePath, std::string xName, std::string yName, std::string zName);

    std::vector<float> TimeSteps(float tMin, float tMax, float dt);

} // namespace SimFramework

#endif //SIMINTERFACE_UTILITIES_H
