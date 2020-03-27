#ifndef FRAMEWORK_SANDBOXMASSSPRINGDAMPER_H
#define FRAMEWORK_SANDBOXMASSSPRINGDAMPER_H


#include <iostream>
#include <fstream>
#include "SimModels/MassSpringDamper1D.h"


void SandboxMassSpringDamper()
{
    // Set up system
    Models::MassSpringDamper1D sys(1.f, 4.f, 0.2);
//    sys.SetLogOutputFile("tmpOut.csv");
    Models::MassSpringDamperBlocks blocks = sys.Blocks();
    sys.Initialise(0);

    int counter = 0;

    for (float t = 0.f; t <= 300.f; t += 0.1) {

        if (counter == 100)
        {
            blocks.InputForceBlock->WriteValue(10.f);
        }

        sys.Update(t);

        std::cout << "t: " << t << ", x: " << blocks.MassPositionBlock->ReadValue() << ", dx/dt: " << blocks.MassVelocityBlock->ReadValue() << std::endl;

        counter ++;
    }
}

#endif //FRAMEWORK_SANDBOXMASSSPRINGDAMPER_H
