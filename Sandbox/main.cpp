//#include "SandboxMassSpringDamper.h"
//#include "SandboxEngine.h"
//#include "SandboxFullModel.h"

#include "SimModels/LockupClutch.h"

int main() {

//    SandboxMassSpringDamper();
//    SandboxEngine();
//    SandboxTransmission();
//    SandboxVehicleStandalone();
//    SandboxFullModel();

    Models::LockupClutch clutch;

    clutch.SetGearRatio(5);



    return 0;
}
