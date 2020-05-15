#ifndef FRAMEWORK_REPORTRESULTS_RATIOS_H
#define FRAMEWORK_REPORTRESULTS_RATIOS_H

#include <vector>
#include "SimModels/Vehicle.h"


void ReportResults_Ratios()
{
    // Parameters
    Models::VehicleParameters baseParameters;
    baseParameters.EngineJSON = "../Sandbox/Data/2L_Turbo_Gasoline.json";
    baseParameters.EngineInitialSpeed = 200.f;
    baseParameters.EngineInertia = 1.f;
    baseParameters.EngineViscousConstant = 0.4;
    baseParameters.GearshiftLag = 1.f;
    baseParameters.ClutchEngagementSpeed = 100.f;
    baseParameters.PullawayClutchMinValue = 0.2;
    baseParameters.ClutchTorqueCapacity = 1000.f;
    baseParameters.GearRatios = {12.f, 8.f, 5.33, 3.555, 2.37};
    baseParameters.TransmissionInertia = 2.f;
    baseParameters.TransmissionViscousFriction = 0.1;
    baseParameters.PeakBrakeForce = 1000.f;
    baseParameters.TyreRadius = 0.2;
    baseParameters.PeakTyreForceScale = 1.f;
    baseParameters.RoadJSON = "../Sandbox/Data/flatRoad.json";
    baseParameters.InitialPosition = 0.f;
    baseParameters.InitialVelocity = 0.f;
    baseParameters.Mass = 1500.f;
    baseParameters.Cd = 0.3;
    baseParameters.A = 3.f;
    baseParameters.rho = 1.225;
    baseParameters.RollingResistance = 0.015;
    baseParameters.Units = Models::EUnitSystem::e_Imperial;
    baseParameters.FuelDensity = 0.7489;
    baseParameters.LogFrequency = 5;
    baseParameters.LogOutputFile = "../Sandbox/Data/Results/Ratio_Base.csv";


    // Initialise base vehicle
    Models::Vehicle baseVehicle;
    baseVehicle.SetParameters(baseParameters);
    Models::VehicleBlocks baseBlocks = baseVehicle.Blocks();

    // Initialise short gears vehicle
    Models::VehicleParameters shortParameters = baseParameters;
    for (auto& ratio : shortParameters.GearRatios)
    {
        ratio *= 1.2;
    }
    shortParameters.LogOutputFile = "../Sandbox/Data/Results/Ratio_Short.csv";
    Models::Vehicle shortVehicle;
    shortVehicle.SetParameters(shortParameters);
    Models::VehicleBlocks shortBlocks = shortVehicle.Blocks();

    // Initialise long gears vehicle
    Models::VehicleParameters longParameters = baseParameters;
    for (auto& ratio : longParameters.GearRatios)
    {
        ratio /= 1.2;
    }
    longParameters.LogOutputFile = "../Sandbox/Data/Results/Ratio_Long.csv";
    Models::Vehicle longVehicle;
    longVehicle.SetParameters(longParameters);
    Models::VehicleBlocks longBlocks = longVehicle.Blocks();

    // Initialise vector of vehicles
    std::vector<Models::Vehicle*> vehicles = {&shortVehicle, &baseVehicle, &longVehicle};
    std::vector<Models::VehicleBlocks*> blocks = {&shortBlocks, &baseBlocks, &longBlocks};

    // Initialise
    for (auto v : vehicles)
    {
        v->Initialise(0.f);
        v->ShiftUp();
    }

    float throttle = 1.f;
    float brake = 0.f;

    // Write throttle and brake values
    for (auto b : blocks)
    {
        b->InThrottle->WriteValue(throttle);
        b->InBrakePressure->WriteValue(brake);
    }

    float dt = 0.1;
    int counter = 1;
    for (float t = 0.f; t <= 400.f; t += dt) {

        if (counter == 200)
        {
            for (auto v : vehicles)
            {
                v->ShiftUp();
            }
        }

        if (counter == 500)
        {
            for (auto v : vehicles)
            {
                v->ShiftUp();
            }
        }

        if (counter == 800)
        {
            for (auto v : vehicles)
            {
                v->ShiftUp();
            }
        }

        if (counter == 1200)
        {
            for (auto v : vehicles)
            {
                v->ShiftUp();
            }
        }

//        if (counter == 2000)
//        {
//            for (auto b : blocks)
//            {
//                b->InThrottle->WriteValue(0.25);
//            }
//            for (auto v : vehicles)
//            {
////                b->InThrottle->WriteValue(0.);
//                v->ShiftDown();
//            }
//        }

        if (counter == 2200)
        {
            for (auto b : blocks)
            {
                b->InThrottle->WriteValue(0.25);
            }
            for (auto v : vehicles)
            {
                v->ShiftDown();
            }
        }

        if (counter == 2600)
        {
            for (auto v : vehicles)
            {
                v->ShiftDown();
            }
        }

        if (counter == 3000)
        {
            for (auto v : vehicles)
            {
                v->ShiftDown();
            }
        }

        if (counter == 3400)
        {
            for (auto v : vehicles)
            {
                v->ShiftDown();
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

#endif //FRAMEWORK_REPORTRESULTS_RATIOS_H
