#ifndef SIMINTERFACE_UTILITIES_H
#define SIMINTERFACE_UTILITIES_H

#include <vector>
#include <string>

#include "Eigen/Dense"

#include "Interpolation.h"


// TODO: consider putting in utility class or namespace
namespace SimFramework {


    // Unit conversions
    float RadiansPerSecondToRPM(float radiansPerSecond);
    float RPMToRadiansPerSecond(float RPM);
    const float pi();

    Table3D ReadTableJSON(std::string JSONFilePath, std::string xName, std::string yName, std::string zName);

    std::vector<float> TimeSteps(float tMin, float tMax, float dt);


    const std::string ToString(const Eigen::VectorXf& vector);
    const std::string ToString(const float& value);
    const std::string ToString(const int& value);
    const std::string ToString(const bool& value);
    const std::string ToString(const std::string& string);

} // namespace SimFramework

#endif //SIMINTERFACE_UTILITIES_H
