#ifndef FRAMEWORK_SANDBOXFULLMODEL_H
#define FRAMEWORK_SANDBOXFULLMODEL_H

#include <iostream>
#include <fstream>
#include "SimFramework/Utilities.h"
#include "SimModels/Vehicle.h"

void SandboxFullModel() {
    // Set up system
    Models::VehicleParameters vehicleParameters;
    vehicleParameters.EngineJSON = "../Sandbox/Data/2L_Turbo_Gasoline.json";
    vehicleParameters.RoadJSON = "../Sandbox/Data/road1.json";
    vehicleParameters.Mass = 1500.f;
    vehicleParameters.GearshiftLag = 0.75;
    vehicleParameters.A = 2.5;
    vehicleParameters.ClutchStiffness = 50.f;
    vehicleParameters.EngineViscousConstant = 0.05;
    vehicleParameters.EngineInertia = 0.2f;
    vehicleParameters.LogFrequency = 1;
    vehicleParameters.LogOutputFile = "LogOut.csv";

    vehicleParameters.GearRatios = {15.f};

    Models::Vehicle vehicle;
    vehicle.SetParameters(vehicleParameters);
    Models::VehicleBlocks blocks = vehicle.Blocks();
    blocks.InThrottle->WriteValue(0.f);
    vehicle.Initialise(0);
    blocks.InThrottle->WriteValue(0.f);


    float dt = 0.1;
    int counter = 1;
    for (float t = 0.f; t <= 500.f; t += dt) {



        if (counter == 200) {
            blocks.InThrottle->WriteValue(1.f);
            vehicle.ShiftUp();
        }

//        if (counter % 400 == 0) {
//            if (counter < 2500) {
//                vehicle.ShiftUp();
//            } else {
//                blocks.InThrottle->WriteValue(0.05);
//
//                vehicle.ShiftDown();
//            }
//        }

        vehicle.Update(t);
        std::cout << "t: " << t
                  <<", Car Pos: " << SimFramework::ToString(blocks.OutCoordinates->ReadValue())
                  << ", Car Vel: " << blocks.OutVelocity->ReadValue()
                  << ", Engine Speed: " << blocks.OutEngineSpeed->ReadValue()
                  << ", Road Gradient: " << blocks.OutGradient->ReadValue()
                << ", Current Gear: " << blocks.OutCurrentGear->ReadValue()
                << std::endl;

        counter++;
    }
}

#endif //FRAMEWORK_SANDBOXFULLMODEL_H
