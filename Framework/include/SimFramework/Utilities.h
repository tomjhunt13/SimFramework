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


    namespace Internal {
        template<typename ElementaryType>
        static const std::string ToString(const ElementaryType &value) {
            std::stringstream ss;
            ss << value;
            std::string formattedFloat = ss.str();

            return formattedFloat;
        };
    }

    static const std::string ToString(const Eigen::VectorXf& vector)
    {
        std::string formattedVector = "";

        for (auto element : vector)
        {
            std::stringstream ss;
            ss << element;
            formattedVector.append(ss.str() + ", ");
        };

        return formattedVector;
    };

    static const std::string ToString(const float& value)
    {
        return Internal::ToString<float>(value);
    }

    static const std::string ToString(const int& value)
    {
        return Internal::ToString<int>(value);
    }

    static const std::string ToString(const bool& value)
    {
        return Internal::ToString<bool>(value);
    }

    static const std::string ToString(const std::string& string)
    {
        return string;
    };

} // namespace SimFramework

#endif //SIMINTERFACE_UTILITIES_H
