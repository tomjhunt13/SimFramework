#ifndef FRAMEWORK_REPORTRESULTS_GRADIENT_H
#define FRAMEWORK_REPORTRESULTS_GRADIENT_H

#include <vector>
#include <cmath>
#include <iostream>
#include "SimFramework/Utilities.h"
#include "ReportResults_VehicleDynamics.h"


float Wavey(float x)
{
    float H = 2.5;
    float lambda = 100;
    float phi = SimFramework::pi() / 2.f;

    return std::atan(2 * x / 50);
//    return std::atan(((2 * SimFramework::pi() / lambda) * H * std::cos(((2 * SimFramework::pi() * x) / lambda) + phi)));
}

void ReportResults_Gradient()
{

    // Parameters
    Report::TestVehicleDynamicsParameters baseParameters;
    baseParameters.initialPosition = 0.f;
    baseParameters.initialVelocity = 0.f;
    baseParameters.mass = 1000;
    baseParameters.Cd = 0.f;
    baseParameters.A = 1;
    baseParameters.rho = 1;
    baseParameters.rollingResistance = 0.f;
    baseParameters.PeakBrakeForce = 5000;
    baseParameters.g = 10;

    // Logging
    baseParameters.LogOutputFile = "../Sandbox/Data/Results/Gradient_Base.csv";
    baseParameters.LogFrequency = 1;


    // Initialise base vehicle
    Report::TestVehicleDynamics baseVehicle;
    baseVehicle.SetParameters(baseParameters);
    Report::TestVehicleDynamicsBlocks baseBlocks = baseVehicle.Blocks();

    baseVehicle.Initialise(0.f);

    float dt = 0.1;
    int counter = 1;

    float tenDeg = 0.174532925199433;
    float grad = -0.1;
    float force = 0;

    baseBlocks.Gradient->WriteValue(grad);

    for (float t = 0.f; t <= 100.f; t += dt) {

        if (counter == 200)
        {
            grad = 0;
        }

        if (counter == 400)
        {
            grad = 0.1;
        }

        if (counter == 800)
        {
            force = 10000.f * std::sin(0.1);
        }


        baseBlocks.Gradient->WriteValue(grad);
        baseBlocks.TyreForce->WriteValue(force);

        baseVehicle.Update(t);

        counter++;
    }

}

#endif //FRAMEWORK_REPORTRESULTS_GRADIENT_H
