#ifndef FRAMEWORK_ROAD_H
#define FRAMEWORK_ROAD_H

#include <vector>
#include <string>

#include "Eigen/Dense"

#include "SimFramework/Framework.h"

namespace Models {

    struct RoadResult {
        float Gradient = 0;
        Eigen::Vector2f Position;
    };

    struct RoadSegment {
        Eigen::Vector2f P1;
        Eigen::Vector2f P2;
        float Gradient;
        float Length;
    };


    class Road {
    public:
        void SetProfile(std::string roadJSONFilepath);
        RoadResult Evaluate(float arcLength);

    private:
        std::vector<float> m_CumulativeLength;
        std::vector<RoadSegment> m_Segments;
    };


    namespace Internal {
        RoadResult EvaluateRoad(float arcLength, std::vector<float>& cumulativeLength, std::vector<RoadSegment>& segments);
    }

}; // namespace Models


#endif //FRAMEWORK_ROAD_H
