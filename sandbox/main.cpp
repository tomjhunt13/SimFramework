#include <iostream>
#include <fstream>
#include <string>

#include "SimFramework/Components.h"
#include "SimModels/MassSpringDamper1D.h"

#include "SimModels/EngineStandalone.h"


int main() {

    std::string engineJSON = "/Users/tom/Documents/University/Y4_S2/Data/Engine/2L_Turbo_Gasoline.json";

    Models::EngineStandalone eng;
    eng.SetEngineParameters(engineJSON, 1.f, 0.2);

    SimFramework::Input<float>* throttleIn = eng.InputThrottleBlock();
    SimFramework::Input<float>* loadIn = eng.InputLoadBlock();
    SimFramework::Output<float>* speedOut = eng.OutputSpeedBlock();
    eng.Initialise(0.f);

    throttleIn->WriteValue(0.5);
    loadIn->WriteValue(100.f);


    std::ofstream myfile;
    myfile.open ("tmpOut.csv", std::ios::out);

    int counter = 0;
    int nSamplesPerSecond = 4;
    float dt = 1.f  / nSamplesPerSecond;

    for (float t = 0.f; t <= 80.f; t += dt) {

        if (counter == (nSamplesPerSecond * 10) )
        {
            loadIn->WriteValue(150.f);
        }

        if (counter == (nSamplesPerSecond * 30))
        {
            throttleIn->WriteValue(0.99);
        }

        if (counter == (nSamplesPerSecond * 50))
        {
            loadIn->WriteValue(200.f);
        }

        eng.Update(t);

        myfile << t << ", " << speedOut->ReadValue() << std::endl;
        std::cout << "t: " << t << ", omega: " << speedOut->ReadValue() << std::endl;

        counter ++;
    }

    myfile.close();

    return 0;
}
