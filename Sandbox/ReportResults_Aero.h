#ifndef FRAMEWORK_REPORTRESULTS_AERO_H
#define FRAMEWORK_REPORTRESULTS_AERO_H

#include <vector>
#include "SimModels/Vehicle.h"



void ReportResults_Aero()
{
    // Parameters
    Models::VehicleParameters baseParameters;
    baseParameters.EngineJSON = "../Sandbox/Data/2L_Turbo_Gasoline.json";
    baseParameters.EngineInitialSpeed = 250.f;
    baseParameters.EngineInertia = 1.f;
    baseParameters.EngineViscousConstant = 0.f;
    baseParameters.GearshiftLag = 0.f;
    baseParameters.ClutchEngagementSpeed = 100.f;
    baseParameters.PullawayClutchMinValue = 0.2;
    baseParameters.ClutchTorqueCapacity = 1000.f;
    baseParameters.GearRatios = {2.f};
    baseParameters.TransmissionInertia = 2.f;
    baseParameters.TransmissionViscousFriction = 0.f;
    baseParameters.PeakBrakeForce = 1000.f;
    baseParameters.TyreRadius = 0.2;
    baseParameters.PeakTyreForceScale = 1.f;
    baseParameters.RoadJSON = "../Sandbox/Data/flatRoad.json";
    baseParameters.InitialPosition = 0.f;
    baseParameters.InitialVelocity = 26.f;
    baseParameters.Mass = 1500.f;
    baseParameters.Cd = 0.75;
    baseParameters.A = 1.f;
    baseParameters.rho = 1.f;
    baseParameters.RollingResistance = 0.f;
    baseParameters.Units = Models::EUnitSystem::e_Imperial;
    baseParameters.FuelDensity = 0.7489;
    baseParameters.LogFrequency = 10;
    baseParameters.LogOutputFile = "../Sandbox/Data/Results/Aero_Base.csv";


    // Initialise base vehicle
    Models::Vehicle baseVehicle;
    baseVehicle.SetParameters(baseParameters);
    Models::VehicleBlocks baseBlocks = baseVehicle.Blocks();

    // Initialise high drag vehicle
    Models::VehicleParameters highDragParameters = baseParameters;
    highDragParameters.A = 1.f;
    highDragParameters.Cd = 1.f;
    highDragParameters.LogOutputFile = "../Sandbox/Data/Results/Aero_HighDrag.csv";
    Models::Vehicle highDragVehicle;
    highDragVehicle.SetParameters(highDragParameters);
    Models::VehicleBlocks highDragBlocks = highDragVehicle.Blocks();

    // Initialise very high drag vehicle
    Models::VehicleParameters veryHighDragParameters = baseParameters;
    veryHighDragParameters.A = 1.f;
    veryHighDragParameters.Cd = 1.25;
    veryHighDragParameters.LogOutputFile = "../Sandbox/Data/Results/Aero_VeryHighDrag.csv";
    Models::Vehicle veryHighDragVehicle;
    veryHighDragVehicle.SetParameters(veryHighDragParameters);
    Models::VehicleBlocks veryHighDragBlocks = veryHighDragVehicle.Blocks();

    // Initialise low drag vehicle
    Models::VehicleParameters lowDragParameters = baseParameters;
    lowDragParameters.A = 1.f;
    lowDragParameters.Cd = 0.5;
    lowDragParameters.LogOutputFile = "../Sandbox/Data/Results/Aero_LowDrag.csv";
    Models::Vehicle lowDragVehicle;
    lowDragVehicle.SetParameters(lowDragParameters);
    Models::VehicleBlocks lowDragBlocks = lowDragVehicle.Blocks();

    // Initialise very light vehicle
    Models::VehicleParameters veryLowDragParameters = baseParameters;
    veryLowDragParameters.A = 1.f;
    veryLowDragParameters.Cd = 0.25;
    veryLowDragParameters.LogOutputFile = "../Sandbox/Data/Results/Aero_VeryLowDrag.csv";
    Models::Vehicle veryLowDragVehicle;
    veryLowDragVehicle.SetParameters(veryLowDragParameters);
    Models::VehicleBlocks veryLowDragBlocks = veryLowDragVehicle.Blocks();

    // Initialise vector of vehicles
    std::vector<Models::Vehicle*> vehicles = {&veryLowDragVehicle, &lowDragVehicle, &baseVehicle, &highDragVehicle, &veryHighDragVehicle};
    std::vector<Models::VehicleBlocks*> blocks = {&veryLowDragBlocks, &lowDragBlocks, &baseBlocks, &highDragBlocks, &veryHighDragBlocks};

    // Initialise
    for (auto v : vehicles)
    {
        v->Initialise(0.f);
        v->ShiftUp();
    }

    // Write throttle and brake values
    for (auto b : blocks)
    {
        b->InThrottle->WriteValue(1.f);
        b->InBrakePressure->WriteValue(0.f);
    }

    float dt = 0.1;
    int counter = 1;
    for (float t = 0.f; t <= 100.f; t += dt) {



        // Update
        for (auto v : vehicles)
        {
            v->Update(t);
        }

        counter++;
    }

}

#endif //FRAMEWORK_REPORTRESULTS_AERO_H
