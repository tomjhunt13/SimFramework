#ifndef FRAMEWORK_REPORTRESULTS_AERO_H
#define FRAMEWORK_REPORTRESULTS_AERO_H

#include <vector>
#include "ReportResults_VehicleDynamics.h"


void ReportResults_Aero()
{

    // Parameters
    Report::TestVehicleDynamicsParameters baseParameters;
    baseParameters.initialPosition = 0;
    baseParameters.initialVelocity = 0;
    baseParameters.mass = 1500;
    baseParameters.Cd = 1;
    baseParameters.A = 1;
    baseParameters.rho = 1;
    baseParameters.rollingResistance = 0.f;
    baseParameters.PeakBrakeForce = 5000;

    // Logging
    baseParameters.LogOutputFile = "../Sandbox/Data/Results/Aero_Base.csv";
    baseParameters.LogFrequency = 1;


    // Initialise base vehicle
    Report::TestVehicleDynamics baseVehicle;
    baseVehicle.SetParameters(baseParameters);
    Report::TestVehicleDynamicsBlocks baseBlocks = baseVehicle.Blocks();

    // Initialise high drag vehicle
    Report::TestVehicleDynamicsParameters highDragParameters = baseParameters;
    highDragParameters.Cd = 2;
    highDragParameters.LogOutputFile = "../Sandbox/Data/Results/Aero_HighDrag.csv";
    Report::TestVehicleDynamics highDragVehicle;
    highDragVehicle.SetParameters(highDragParameters);
    Report::TestVehicleDynamicsBlocks highDragBlocks = highDragVehicle.Blocks();

    // Initialise very high drag vehicle
    Report::TestVehicleDynamicsParameters veryHighDragParameters = baseParameters;
    veryHighDragParameters.Cd = 4;
    veryHighDragParameters.LogOutputFile = "../Sandbox/Data/Results/Aero_VeryHighDrag.csv";
    Report::TestVehicleDynamics veryHighDragVehicle;
    veryHighDragVehicle.SetParameters(veryHighDragParameters);
    Report::TestVehicleDynamicsBlocks veryHighDragBlocks = veryHighDragVehicle.Blocks();

    // Initialise low drag vehicle
    Report::TestVehicleDynamicsParameters lowDragParameters = baseParameters;
    lowDragParameters.Cd = 0.5;
    lowDragParameters.LogOutputFile = "../Sandbox/Data/Results/Aero_LowDrag.csv";
    Report::TestVehicleDynamics lowDragVehicle;
    lowDragVehicle.SetParameters(lowDragParameters);
    Report::TestVehicleDynamicsBlocks lowDragBlocks = lowDragVehicle.Blocks();

    // Initialise very low drag vehicle
    Report::TestVehicleDynamicsParameters veryLowDragParameters = baseParameters;
    veryLowDragParameters.Cd = 0.25;
    veryLowDragParameters.LogOutputFile = "../Sandbox/Data/Results/Aero_VeryLowDrag.csv";
    Report::TestVehicleDynamics veryLowDragVehicle;
    veryLowDragVehicle.SetParameters(veryLowDragParameters);
    Report::TestVehicleDynamicsBlocks veryLowDragBlocks = veryLowDragVehicle.Blocks();

    // Initialise vector of vehicles
    std::vector<Report::TestVehicleDynamics*> vehicles = {&veryLowDragVehicle, &lowDragVehicle, &baseVehicle, &highDragVehicle, &veryHighDragVehicle};
    std::vector<Report::TestVehicleDynamicsBlocks*> blocks = {&veryLowDragBlocks, &lowDragBlocks, &baseBlocks, &highDragBlocks, &veryHighDragBlocks};

    // Initialise
    for (auto v : vehicles)
    {
        v->Initialise(0.f);
    }

    // Write throttle and brake values
    for (auto b : blocks)
    {
        b->TyreForce->WriteValue(10000.f);
        b->Gradient->WriteValue(0.f);
        b->BrakePedal->WriteValue(0.f);
    }

    float dt = 0.1;
    int counter = 1;
    for (float t = 0.f; t <= 400.f; t += dt) {

        if (counter == 2000) {
            for (auto b : blocks)
            {
                b->TyreForce->WriteValue(0.f);
//                b->BrakePedal->WriteValue(1.f);
            }
        }

        // Update
        for (auto v : vehicles)
        {
            v->Update(t);
        }

        counter++;
    }

}

#endif //FRAMEWORK_REPORTRESULTS_AERO_H
