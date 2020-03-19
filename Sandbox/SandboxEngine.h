#ifndef FRAMEWORK_SANDBOXENGINE_H
#define FRAMEWORK_SANDBOXENGINE_H

#include <iostream>
#include <fstream>
#include <string>

#include "SimModels/EngineStandalone.h"


void SandboxEngine()
{

    std::string engineJSON = "/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json";

    Models::EngineStandalone eng(engineJSON, 300);

    Models::EngineBlocks blocks = eng.Blocks();
    eng.Initialise(0.f);

    blocks.InputThrottleBlock->WriteValue(0.5);
    blocks.InputLoadBlock->WriteValue(100.f);


    std::ofstream myfile;
    myfile.open ("tmpOut.csv", std::ios::out);

    int counter = 0;
    int nSamplesPerSecond = 4;
    float dt = 1.f  / nSamplesPerSecond;

    for (float t = 0.f; t <= 80.f; t += dt) {

        if (counter == (nSamplesPerSecond * 10) )
        {
            blocks.InputLoadBlock->WriteValue(150.f);
        }

        if (counter == (nSamplesPerSecond * 30))
        {
            blocks.InputThrottleBlock->WriteValue(0.99);
        }

        if (counter == (nSamplesPerSecond * 50))
        {
            blocks.InputLoadBlock->WriteValue(200.f);
        }

        eng.Update(t);

        myfile << t << ", " << blocks.OutputSpeedBlock->ReadValue() << std::endl;
        std::cout << "t: " << t << ", omega: " << blocks.OutputSpeedBlock->ReadValue() << std::endl;

        counter ++;
    }

    myfile.close();

}

#endif //FRAMEWORK_SANDBOXENGINE_H
