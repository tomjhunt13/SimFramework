#ifndef FRAMEWORK_SANDBOXFULLMODEL_H
#define FRAMEWORK_SANDBOXFULLMODEL_H

#include <iostream>
#include <fstream>
#include "SimModels/Vehicle.h"

void SandboxFullModel()
{
    // Set up system
    Models::VehicleParameters vehicleParameters;
    vehicleParameters.GearshiftLag = 2.f;
    vehicleParameters.A = 3.f;

    Models::Vehicle vehicle(vehicleParameters);
    Models::VehicleBlocks blocks = vehicle.Blocks();
    blocks.InThrottle->WriteValue(1.f);
    vehicle.Initialise(0);
    blocks.InThrottle->WriteValue(1.f);


    // Set up file writing
    std::ofstream myfile;
    myfile.open ("tmpOut.csv", std::ios::out);



    float dt = 0.05;
    int counter = 1;
    for (float t = 0.f; t <= 200.f; t += dt) {

        if (counter % 100 == 0)
        {
            if (counter < 850)
            {
                vehicle.ShiftUp();
            }

            else
            {
                vehicle.ShiftDown();
            }
        }

        if (counter == 1400)
        {
            blocks.InThrottle->WriteValue(0.f);
            blocks.InBrakePressure->WriteValue(1.f);
        }

        if (counter == 1800)
        {
            blocks.InThrottle->WriteValue(0.25f);
            blocks.InBrakePressure->WriteValue(0.f);
        }

        if (counter == 2000)
        {
            blocks.InThrottle->WriteValue(0.5f);
            blocks.InBrakePressure->WriteValue(0.f);
        }

        if (counter == 2200)
        {
            blocks.InThrottle->WriteValue(0.75f);
            blocks.InBrakePressure->WriteValue(0.f);
        }

        if (counter == 2400)
        {
            blocks.InThrottle->WriteValue(0.25f);
            blocks.InBrakePressure->WriteValue(0.f);
        }

        if (counter == 2600)
        {
            blocks.InThrottle->WriteValue(1.f);
            blocks.InBrakePressure->WriteValue(0.f);
        }


        vehicle.Update(t);

        myfile << t << ", " << blocks.OutPosition->ReadValue() << ", " << blocks.OutVelocity->ReadValue() << ", " << blocks.OutEngineSpeed->ReadValue() << std::endl;
        std::cout << "t: " << t << ", Car Pos: " << blocks.OutPosition->ReadValue() << ", Car Vel: " << blocks.OutVelocity->ReadValue() << ", Engine Speed: " << blocks.OutEngineSpeed->ReadValue()<< std::endl;

        counter ++;
    }

    myfile.close();
}

#endif //FRAMEWORK_SANDBOXFULLMODEL_H
