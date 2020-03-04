#include <iostream>
#include <fstream>

#include "SimFramework/Components.h"
#include "SimModels/MassSpringDamper1D.h"


int main() {


    Models::MassSpringDamper1D sys(1.f, 4.f, 0.2);
    sys.Initialise(0);

    SimFramework::Input<float>* simIn = sys.InputForceBlock();
    SimFramework::Output<float>* posOut = sys.MassPositionBlock();
    SimFramework::Output<float>* velOut = sys.MassVelocityBlock();

    simIn->WriteValue(0.f);

    std::ofstream myfile;

    // ios::out indicates writing, ios::app indicates appending to existing
    myfile.open ("tmpOut.csv", std::ios::out | std::ios::app);

    int counter = 0;

    for (float t = 0.f; t <= 300.f; t += 0.001) {

        if (counter == 30000)
        {
            simIn->WriteValue(10.f);
        }

        sys.Update(t);

        myfile << t << ", " << posOut->ReadValue() << ", " << velOut->ReadValue() << std::endl;
        std::cout << "t: " << t << ", x: " << posOut->ReadValue() << ", dx/dt: " << velOut->ReadValue() << std::endl;

        counter ++;
    }

    myfile.close();

    return 0;
}
