#include <iostream>
#include <fstream>
#include <string>

#include "SimModels/TransmissionStandalone.h"


int main() {


    Models::Transmission transmission;

    SimFramework::Input<float>* ClutchInBlock = transmission.ClutchInBlock();
    SimFramework::Input<float>* TyreInBlock = transmission.TyreInBlock();
    SimFramework::Output<float>* ClutchOutBlock = transmission.ClutchOutBlock();
    SimFramework::Output<float>* TyreOutBlock = transmission.TyreOutBlock();
    Models::LinearTrigger* TriggerBlock = transmission.TriggerBlock();

    transmission.Initialise(0);
    ClutchInBlock->WriteValue(10);
    TyreInBlock->WriteValue(5);

    std::ofstream myfile;
    myfile.open ("tmpOut.csv", std::ios::out);


    float dt = 0.05;
    int counter = 0;
    for (float t = 0.f; t <= 80.f; t += dt) {

        if (counter % 100 == 0)
        {
            TriggerBlock->Trigger();
        }

        transmission.Update(t);

        myfile << t << ", " << ClutchOutBlock->ReadValue() << std::endl;
        std::cout << "t: " << t << ", val: " << ClutchOutBlock->ReadValue() << std::endl;

        counter ++;
    }

    myfile.close();

    return 0;
}
