#include "SimModels/Road.h"

#include <cmath>
#include <fstream>
#include "nlohmann/json.hpp"

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
        this->m_Segments.resize(x.size() - 2);
        this->m_CumulativeLength.resize(x.size() - 2);

        for (int i = 0; i < this->m_Segments.size(); i++)
        {
            this->m_Segments[i].P1 = {x[i], y[i]};
            this->m_Segments[i].P2 = {x[i+1], y[i+1]};

            float P1P2[2] = {x[i+1] - x[i], y[i+1] - y[i]};

            this->m_Segments[i].Gradient = std::atan(P1P2[1] / P1P2[0]);
            this->m_Segments[i].Length = std::sqrt(P1P2[0] * P1P2[0] + P1P2[1] * P1P2[1]);

            if (i == 0)
            {
                this->m_CumulativeLength[0] = this->m_Segments[0].Length;
            }
            else
            {
                this->m_CumulativeLength[i] = this->m_CumulativeLength[i-1] + this->m_Segments[i].Length;
            }
        };
    };


}; // namespace Models
