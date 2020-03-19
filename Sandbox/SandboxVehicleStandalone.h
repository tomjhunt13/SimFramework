#ifndef FRAMEWORK_SANDBOXVEHICLESTANDALONE_H
#define FRAMEWORK_SANDBOXVEHICLESTANDALONE_H

#include <iostream>
#include <fstream>
#include "SimModels/VehicleDynamicsStandalone.h"

void SandboxVehicleStandalone()
{
    Models::VehicleDynamicsParams params;
    params.InitialVelocity = 100;

    Models::VehicleDynamicsStandalone vehicle(params);

    Models::VehicleDynamicsBlocks blocks = vehicle.Blocks();

    vehicle.Initialise(0);
    blocks.InTyreForce->WriteValue(20);;

    std::ofstream myfile;
    myfile.open ("tmpOut.csv", std::ios::out);


    float dt = 0.5;
    int counter = 1;
    for (float t = 0.f; t <= 3000.f; t += dt) {


        vehicle.Update(t);

        myfile << t << ", " << blocks.OutVehiclePosition->ReadValue() << ", " << blocks.OutVehicleSpeed->ReadValue() << std::endl;
        std::cout << "t: " << t << ", position: " << blocks.OutVehiclePosition->ReadValue() << ", speed:" << blocks.OutVehicleSpeed->ReadValue() << std::endl;

        counter ++;
    }

    myfile.close();
}

#endif //FRAMEWORK_SANDBOXVEHICLESTANDALONE_H
