#ifndef FRAMEWORK_REPORTRESULTS_MASS_H
#define FRAMEWORK_REPORTRESULTS_MASS_H

#include <vector>
#include "ReportResults_VehicleDynamics.h"



void ReportResults_Mass()
{
    // Parameters
    Report::TestVehicleDynamicsParameters baseParameters;
    baseParameters.initialPosition = 0;
    baseParameters.initialVelocity = 0;
    baseParameters.mass = 1500;
    baseParameters.Cd = 0.3;
    baseParameters.A = 2.5;
    baseParameters.rho = 1.225;
    baseParameters.rollingResistance = 0.015;

    // Logging
    baseParameters.LogOutputFile = "../Sandbox/Data/Results/Mass_Base.csv";
    baseParameters.LogFrequency = 1;


    // Initialise base vehicle
    Report::TestVehicleDynamics baseVehicle;
    baseVehicle.SetParameters(baseParameters);
    Report::TestVehicleDynamicsBlocks baseBlocks = baseVehicle.Blocks();

    // Initialise heavy vehicle
    Report::TestVehicleDynamicsParameters heavyParameters = baseParameters;
    heavyParameters.mass = 2000.f;
    heavyParameters.LogOutputFile = "../Sandbox/Data/Results/Mass_Heavy.csv";
    Report::TestVehicleDynamics heavyVehicle;
    heavyVehicle.SetParameters(heavyParameters);
    Report::TestVehicleDynamicsBlocks heavyBlocks = heavyVehicle.Blocks();

    // Initialise very heavy vehicle
    Report::TestVehicleDynamicsParameters veryHeavyParameters = baseParameters;
    veryHeavyParameters.mass = 2500.f;
    veryHeavyParameters.LogOutputFile = "../Sandbox/Data/Results/Mass_VeryHeavy.csv";
    Report::TestVehicleDynamics veryHeavyVehicle;
    veryHeavyVehicle.SetParameters(veryHeavyParameters);
    Report::TestVehicleDynamicsBlocks veryHeavyBlocks = veryHeavyVehicle.Blocks();

    // Initialise light vehicle
    Report::TestVehicleDynamicsParameters lightParameters = baseParameters;
    lightParameters.mass = 1000.f;
    lightParameters.LogOutputFile = "../Sandbox/Data/Results/Mass_Light.csv";
    Report::TestVehicleDynamics lightVehicle;
    lightVehicle.SetParameters(lightParameters);
    Report::TestVehicleDynamicsBlocks lightBlocks = lightVehicle.Blocks();

    // Initialise very light vehicle
    Report::TestVehicleDynamicsParameters veryLightParameters = baseParameters;
    veryLightParameters.mass = 500.f;
    veryLightParameters.LogOutputFile = "../Sandbox/Data/Results/Mass_VeryLight.csv";
    Report::TestVehicleDynamics veryLightVehicle;
    veryLightVehicle.SetParameters(veryLightParameters);
    Report::TestVehicleDynamicsBlocks veryLightBlocks = veryLightVehicle.Blocks();

    // Initialise vector of vehicles
    std::vector<Report::TestVehicleDynamics*> vehicles = {&veryLightVehicle, &lightVehicle, &baseVehicle, &heavyVehicle, &veryHeavyVehicle};
    std::vector<Report::TestVehicleDynamicsBlocks*> blocks = {&veryLightBlocks, &lightBlocks, &baseBlocks, &heavyBlocks, &veryHeavyBlocks};

//    // Initialise
//    for (auto v : vehicles)
//    {
//        v->Initialise(0.f);
//    }
//
//    // Write throttle and brake values
//    for (auto b : blocks)
//    {
//        b->TyreForce->WriteValue(1.f);
//        b->InBrakePressure->WriteValue(0.f);
//    }
//
//    float dt = 0.00025;
//    int counter = 1;
//    for (float t = 0.f; t <= 25.f; t += dt) {
//
//        if (counter == 60000) {
//            for (auto b : blocks)
//            {
//                b->InThrottle->WriteValue(0.f);
//                b->InBrakePressure->WriteValue(1.f);
//            }
//            for (auto v : vehicles)
//            {
//                v->ShiftDown();
//            }
//        }
//
//        // Update
//        for (auto v : vehicles)
//        {
//            v->Update(t);
//        }
//
//        counter++;
//    }

}



#endif //FRAMEWORK_REPORTRESULTS_MASS_H
