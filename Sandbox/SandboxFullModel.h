#ifndef FRAMEWORK_SANDBOXFULLMODEL_H
#define FRAMEWORK_SANDBOXFULLMODEL_H

#include <iostream>
#include <fstream>
#include "SimModels/Vehicle.h"

void SandboxFullModel() {
    // Set up system
    Models::VehicleParameters vehicleParameters;
    vehicleParameters.EngineJSON = "../Sandbox/Data/2L_Turbo_Gasoline.json";
    vehicleParameters.RoadJSON = "../Sandbox/Data/road1.json";
    vehicleParameters.Mass = 1500.f;
    vehicleParameters.GearshiftLag = 0.75;
    vehicleParameters.A = 2.5;
    vehicleParameters.ClutchStiffness = 100.f;
    vehicleParameters.EngineViscousConstant = 0.05;
    vehicleParameters.EngineInertia = 0.2f;
    vehicleParameters.LogFrequency = 1;
    vehicleParameters.LogOutputFile = "LogOut.csv";

    Models::Vehicle vehicle;
    vehicle.SetParameters(vehicleParameters);
    Models::VehicleBlocks blocks = vehicle.Blocks();
    blocks.InThrottle->WriteValue(0.f);
    vehicle.Initialise(0);
    blocks.InThrottle->WriteValue(0.f);


    float dt = 0.5;
    int counter = 1;
    for (float t = 0.f; t <= 1400.f; t += dt) {

        if (counter == 200) {
            blocks.InThrottle->WriteValue(1.f);
        }

        if (counter % 400 == 0) {
            if (counter < 2500) {
                vehicle.ShiftUp();
            } else {
                blocks.InThrottle->WriteValue(0.05);

                vehicle.ShiftDown();
            }
        }

//        if (counter == 1400)
//        {
//            blocks.InThrottle->WriteValue(0.f);
//            blocks.InBrakePressure->WriteValue(1.f);
//        }
//
//        if (counter == 1800)
//        {
//            blocks.InThrottle->WriteValue(0.25f);
//            blocks.InBrakePressure->WriteValue(0.f);
//        }
//
//        if (counter == 2000)
//        {
//            blocks.InThrottle->WriteValue(0.5f);
//            blocks.InBrakePressure->WriteValue(0.f);
//        }
//
//        if (counter == 2200)
//        {
//            blocks.InThrottle->WriteValue(0.75f);
//            blocks.InBrakePressure->WriteValue(0.f);
//        }
//
//        if (counter == 2400)
//        {
//            blocks.InThrottle->WriteValue(0.25f);
//            blocks.InBrakePressure->WriteValue(0.f);
//        }
//
//        if (counter == 2600)
//        {
//            blocks.InThrottle->WriteValue(1.f);
//            blocks.InBrakePressure->WriteValue(0.f);
//        }


        vehicle.Update(t);
        std::cout << "t: " << t << ", Car Pos: " << blocks.OutPosition->ReadValue() << ", Car Vel: "
                  << blocks.OutVelocity->ReadValue() << ", Engine Speed: " << blocks.OutEngineSpeed->ReadValue()
                  << std::endl;

        counter++;
    }
}

#endif //FRAMEWORK_SANDBOXFULLMODEL_H
