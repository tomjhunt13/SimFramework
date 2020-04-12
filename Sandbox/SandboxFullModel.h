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
    vehicleParameters.RoadJSON = "../Sandbox/Data/flatRoad.json";
    vehicleParameters.Mass = 1500.f;
    vehicleParameters.GearshiftLag = 0.75;
    vehicleParameters.A = 2.5;
    vehicleParameters.ClutchMaxNormalForce = 500.f;
    vehicleParameters.ClutchTorqueCapacity = 1.f;
    vehicleParameters.EngineInertia = 0.2f;
    vehicleParameters.TransmissionInertia = 4.f;
    vehicleParameters.LogFrequency = 1;
    vehicleParameters.GearRatios = {10};
    vehicleParameters.InitialVelocity = 0;
    vehicleParameters.LogOutputFile = "LogOut.csv";


    Models::Vehicle vehicle;
    vehicle.SetParameters(vehicleParameters);
    Models::VehicleBlocks blocks = vehicle.Blocks();
    blocks.InThrottle->WriteValue(1.f);
    vehicle.Initialise(0);
    blocks.InThrottle->WriteValue(1.f);


    float dt = 0.1;
    int counter = 1;
    for (float t = 0.f; t <= 500.f; t += dt) {



        if (counter == 200) {
            blocks.InThrottle->WriteValue(1.f);
//            vehicle.ShiftUp();
        }

        if (counter % 400 == 0) {
            if (counter < 2500) {
//                vehicle.ShiftUp();
            }
        }

        vehicle.Update(t);
        std::cout << "t: " << t
                  <<", Car Pos: " << SimFramework::ToString(blocks.OutCoordinates->ReadValue())
                  << ", Car Vel: " << blocks.OutLinearVelocity->ReadValue()
                  << ", Engine Speed: " << blocks.OutEngineSpeed->ReadValue()
                  << ", Road Gradient: " << blocks.OutGradient->ReadValue()
                  << ", Fuel Usage: " << blocks.OutFuelCumulative->ReadValue()
                  << ", Fuel Rate: " << blocks.OutFuelFlowRate->ReadValue()
                  << ", Current Gear: " << blocks.OutCurrentGear->ReadValue()
                  << ", Clutch Engaged State: " << blocks.OutClutchLockState->ReadValue()

                << std::endl;

        counter++;
    }
}

#endif //FRAMEWORK_SANDBOXFULLMODEL_H
