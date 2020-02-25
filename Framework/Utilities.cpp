#include "Utilities.h"

// Private includes
#include <cmath>
#include <fstream>
#include "nlohmann/json.hpp"

namespace SimFramework {

    float RadiansPerSecondToRPM(float radiansPerSecond)
    {
        return radiansPerSecond * 9.54929658551;
    };

    float RPMToRadiansPerSecond(float RPM)
    {
        return RPM * 0.10471975512;
    };


    Table3D ReadTableJSON(std::string JSONFilePath, std::string xName, std::string yName, std::string zName)
    {
        // Read JSON and extract data
        std::ifstream fileObject;
        fileObject.open (JSONFilePath);
        nlohmann::json js = nlohmann::json::parse(fileObject);
        std::vector<float> x = js[xName];
        std::vector<float> y = js[yName];
        std::vector<std::vector<float>> z = js[zName];
        fileObject.close();

        return {x, y, z};
    };


    std::vector<float> TimeSteps(float tMin, float tMax, float dt) {

        float tRange = tMax - tMin;

        std::vector<float> timesteps;

        if (tRange > 0.f) {

            if (dt >= tRange) {
                timesteps.push_back(dt);
            } else {
                int nt = std::ceilf(tRange / dt);
                float dtNew = tRange / (float) nt;

                for (int i = 0; i < nt; i++) {
                    timesteps.push_back(dtNew);
                }
            }
        }

        return timesteps;
    };

} // namespace SimFramework