#ifndef FRAMEWORK_SANDBOXMASSSPRINGDAMPER_H
#define FRAMEWORK_SANDBOXMASSSPRINGDAMPER_H


#include <iostream>
#include <fstream>
#include "SimModels/MassSpringDamper1D.h"


void SandboxMassSpringDamper()
{

    Models::MassSpringDamper1D sys(1.f, 4.f, 0.2);
    sys.Initialise(0);

    Models::MassSpringDamperBlocks blocks = sys.Blocks();
    blocks.InputForceBlock->WriteValue(0.f);

    std::ofstream myfile;

    // ios::out indicates writing
    myfile.open ("tmpOut.csv", std::ios::out);

    int counter = 0;

    for (float t = 0.f; t <= 300.f; t += 0.1) {

        if (counter == 100)
        {
            blocks.InputForceBlock->WriteValue(10.f);
        }

        sys.Update(t);

        myfile << t << ", " << blocks.MassPositionBlock->ReadValue() << ", " << blocks.MassVelocityBlock->ReadValue() << std::endl;
        std::cout << "t: " << t << ", x: " << blocks.MassPositionBlock->ReadValue() << ", dx/dt: " << blocks.MassVelocityBlock->ReadValue() << std::endl;

        counter ++;
    }

    myfile.close();
}

#endif //FRAMEWORK_SANDBOXMASSSPRINGDAMPER_H
