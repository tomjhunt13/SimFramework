#include "Utilities.h"

namespace SimFramework {

    float RadiansPerSecondToRPM(float radiansPerSecond)
    {
        return radiansPerSecond * 9.54929658551;
    };

    float RPMToRadiansPerSecond(float RPM)
    {
        return RPM * 0.10471975512;
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

} // namespace Framework