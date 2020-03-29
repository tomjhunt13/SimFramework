#include "SandboxMassSpringDamper.h"
#include "SandboxEngine.h"
#include "SandboxTransmission.h"
#include "SandboxVehicleStandalone.h"
#include "SandboxFullModel.h"

#include <iostream>
#include "SimModels/Road.h"


int main() {

    Models::Road r;
    r.SetProfile("/Users/tom/Documents/University/Y4_S2/Data/Road/road1.json");

    Models::RoadResult res = r.Evaluate(640);

    std::cout << res.Position << std::endl;
    int a = 1;

//    SandboxMassSpringDamper();
//    SandboxEngine();
//    SandboxTransmission();
//    SandboxVehicleStandalone();
//    SandboxFullModel();



    return 0;
}
