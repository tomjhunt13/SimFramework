#include "SimFramework/Utilities.h"

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

    float MetresToMiles(float metres)
    {
        return metres / 1609.34;
    };

    float MetresToKilometers(float metres)
    {
        return metres / 1000.f;
    };

    float SecondsToHours(float seconds)
    {
        return seconds / 3600.f;
    };

    float MetresPerSecondToMPH(float metresPerSecond)
    {
        return metresPerSecond * 2.237136;
    };

    float MetresPerSecondToKPH(float metresPerSecond)
    {
        return metresPerSecond * 3.6;
    };

    float MassToVolume(float mass, float density)
    {
        return mass / density;
    };

    float CentimetresCubedToLitres(float centimetresCubed)
    {
        return centimetresCubed / 1000.f;
    };

    float CentimetresCubedToGallons(float centimetresCubed)
    {
        return centimetresCubed * 0.000219969‬‬;
    };

    const float pi()
    {
        return 3.141592653589793;
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


    std::vector<float> TimeSteps(float tMin, float tMax, float dtMax) {

        float tRange = tMax - tMin;

        std::vector<float> timesteps;

        if (tRange > 0.f) {

            if (dtMax >= tRange) {
                timesteps.push_back(tRange);
            } else {
                int nt = std::ceilf(tRange / dtMax);
                float dtNew = tRange / (float) nt;

                for (int i = 0; i < nt; i++) {
                    timesteps.push_back(dtNew);
                }
            }
        }

        return timesteps;
    };


    template<typename ElementaryType>
    static const std::string InternalToString(const ElementaryType &value) {
        std::stringstream ss;
        ss << value;
        std::string formattedFloat = ss.str();

        return formattedFloat;
    };

    const std::string ToString(const Eigen::VectorXf& vector)
    {
        std::string formattedVector = "";

        for (int i=0; i < vector.size(); i++)
        {
            std::stringstream ss;
            ss << vector[i];
            formattedVector.append(ss.str());

            if (i < (vector.size() - 1))
            {
                formattedVector.append(",");
            }
        };

        return formattedVector;
    };

    const std::string ToString(const float& value)
    {
        return InternalToString<float>(value);
    }

    const std::string ToString(const int& value)
    {
        return InternalToString<int>(value);
    }

    const std::string ToString(const bool& value)
    {
        return InternalToString<bool>(value);
    }

    const std::string ToString(const std::string& string)
    {
        return string;
    };



    CSVWriter::CSVWriter() : m_Filepath("outCSV.csv")
    {
        this->ResetFile();
    };

    void CSVWriter::SetOutputFilepath(std::string filepath)
    {
        this->m_Filepath = filepath;
        this->ResetFile();
    };

    void CSVWriter::SetHeader(std::vector<std::string> names)
    {
        this->m_HeaderNames = names;
        this->AppendRow(names);
    };

    void CSVWriter::AppendRow(std::vector<std::string> values)
    {
        std::string row;
        for (int i = 0; i < values.size(); i++)
        {
            row += values[i];

            if (i < (values.size() - 1))
            {
                row += ",";
            }
        }

        std::ofstream file;
        file.open(this->m_Filepath, std::ios::app);
        file << row << std::endl;
        file.close();
    };

    void CSVWriter::ResetFile()
    {
        std::ofstream file;
        file.open(this->m_Filepath, std::ios::trunc);
        file.close();
    };

} // namespace SimFramework