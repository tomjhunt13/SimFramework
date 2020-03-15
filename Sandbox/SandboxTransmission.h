#ifndef FRAMEWORK_SANDBOXTRANSMISSION_H
#define FRAMEWORK_SANDBOXTRANSMISSION_H

#include <iostream>
#include <fstream>
#include <string>

#include "SimModels/TransmissionStandalone.h"

void SandboxTransmission()
{
    Models::TransmissionStandalone transmission;

    Models::TransmissionBlocks blocks = transmission.Blocks();

    transmission.Initialise(0);
    blocks.ClutchInBlock->WriteValue(12);
    blocks.TyreInBlock->WriteValue(4);

    std::ofstream myfile;
    myfile.open ("tmpOut.csv", std::ios::out);


    float dt = 0.05;
    int counter = 1;
    for (float t = 0.f; t <= 80.f; t += dt) {

        if (counter % 100 == 0)
        {
            if (counter < 850)
            {
                transmission.ShiftUp();
            }

            else
            {
                transmission.ShiftDown();
            }
        }


        transmission.Update(t);

        myfile << t << ", " << blocks.ClutchOutBlock->ReadValue() << ", " << transmission.CurrentGear() << std::endl;
        std::cout << "t: " << t << ", val: " << blocks.ClutchOutBlock->ReadValue() << ", gear:" << transmission.CurrentGear() << std::endl;

        counter ++;
    }

    myfile.close();
}

#endif //FRAMEWORK_SANDBOXTRANSMISSION_H
