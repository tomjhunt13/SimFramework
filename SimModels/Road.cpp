#include "SimModels/Road.h"

#include <cmath>
#include <fstream>
#include "nlohmann/json.hpp"
#include "SimFramework/Interpolation.h"


// TODO: put in interpolation
template <typename ValueType>
ValueType LinearBlend(float alpha, const ValueType& P1, const ValueType& P2)
{
    return (1.f - alpha) * P1 + alpha * P2;
}

template <typename ValueType>
ValueType LinearBlendClamped(float alpha, const ValueType& P1, const ValueType& P2)
{
    if (alpha <= 0)
    {
        return P1;
    }
    else if (alpha >= 1)
    {
        return P2;
    }

    return (1.f - alpha) * P1 + alpha * P2;
}





namespace Models {


    void Road::SetProfile(std::string roadJSONFilepath)
    {
        // Read in vertices
        std::ifstream fileObject;
        fileObject.open (roadJSONFilepath);
        nlohmann::json js = nlohmann::json::parse(fileObject);
        std::vector<float> x = js["x"];
        std::vector<float> y = js["y"];
        fileObject.close();

        // Create edges
        this->m_Segments.resize(x.size() - 1);
        this->m_CumulativeLength.resize(x.size() - 1);

        for (int i = 0; i < this->m_Segments.size(); i++)
        {
            this->m_Segments[i].P1 = {x[i], y[i]};
            this->m_Segments[i].P2 = {x[i+1], y[i+1]};

            float P1P2[2] = {x[i+1] - x[i], y[i+1] - y[i]};

            this->m_Segments[i].Gradient = std::atan(P1P2[1] / P1P2[0]);
            this->m_Segments[i].Length = std::sqrt(P1P2[0] * P1P2[0] + P1P2[1] * P1P2[1]);

            if (i == 0)
            {
                this->m_CumulativeLength[0] = 0;
            }
            else
            {
                this->m_CumulativeLength[i] = this->m_CumulativeLength[i-1] + this->m_Segments[i].Length;
            }
        };
    };

    RoadResult Road::Evaluate(float arcLength)
    {
        return Internal::EvaluateRoad(arcLength, this->m_CumulativeLength, this->m_Segments);
    }

    RoadResult Internal::EvaluateRoad(float arcLength, std::vector<float>& CumulativeLength, std::vector<RoadSegment>& Segments)
    {
        // Get edge indices
        std::vector<int> indices = SimFramework::Internal::NearestIndices(CumulativeLength, arcLength);

        // Get edge info
        int index = 0;
        if (arcLength >= CumulativeLength[indices[1]])
        {
            index = 1;
        }
        RoadSegment& segment = Segments[indices[index]];
        float segParam = (arcLength - CumulativeLength[indices[index]]) / segment.Length;
        return {segment.Gradient, LinearBlendClamped(segParam, segment.P1, segment.P2)};
    };

}; // namespace Models
