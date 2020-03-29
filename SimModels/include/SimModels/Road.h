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

    class Road : public SimFramework::Function {
    public:
        void Configure(const SimFramework::Signal<float>* inArcLength);
        const SimFramework::Signal<Eigen::Vector2f>* OutPosition() const;
        const SimFramework::Signal<float>* OutGradient() const;

        void SetProfile(std::string roadJSONFilepath);
        std::vector<const SimFramework::SignalBase*> InputSignals() const override;
        std::vector<const SimFramework::SignalBase*> OutputSignals() const override;
        void Update() override;

    private:
        // Parameters
        std::vector<float> m_CumulativeLength;
        std::vector<RoadSegment> m_Segments;

        // Signals
        const SimFramework::Signal<float>* m_InArcLength;
        SimFramework::Signal<Eigen::Vector2f> m_OutPosition;
        SimFramework::Signal<float> m_OutGradient;
    };

    namespace Internal {
        RoadResult EvaluateRoad(float arcLength, std::vector<float>& cumulativeLength, std::vector<RoadSegment>& segments);
    }

}; // namespace Models


#endif //FRAMEWORK_ROAD_H
