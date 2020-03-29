#ifndef FRAMEWORK_ROAD_H
#define FRAMEWORK_ROAD_H

#include <vector>
#include <string>

#include "SimFramework/Interpolation.h"
#include "SimFramework/Framework.h"


namespace Models {

    struct RoadSegment {
        RoadSegment() : P1(2), P2(2) {};

        std::vector<float> P1;
        std::vector<float> P2;
        float Gradient;
        float Length;
    };



    class Road {
    public:

        void SetProfile(std::string roadJSONFilepath);

    private:

        std::vector<float> m_CumulativeLength;
        std::vector<RoadSegment> m_Segments;


    };

}; // namespace Models


#endif //FRAMEWORK_ROAD_H
