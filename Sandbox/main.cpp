//#include "SandboxMassSpringDamper.h"
//#include "SandboxEngine.h"
//#include "SandboxFullModel.h"

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"
#include "SimModels/LockupClutch.h"

int main() {
    SimFramework::ConstantBlock<float> c1;
    SimFramework::ConstantBlock<float> c2;
    SimFramework::ConstantBlock<float> c3;

    c1.Configure(0.f);
    c2.Configure(4.f);
    c3.Configure(8.f);

    SimFramework::Switch<float> switchBlock;
    switchBlock.Configure({c1.OutSignal(), c2.OutSignal(), c3.OutSignal()}, 0);

    c1.Initialise(0.f);
    c2.Initialise(0.f);
    c3.Initialise(0.f);

    switchBlock.Update();
    float kjhb = switchBlock.OutSignal()->Read();

    switchBlock.SetIndex(1);
    switchBlock.Update();
    kjhb = switchBlock.OutSignal()->Read();

    switchBlock.SetIndex(2);
    switchBlock.Update();
    kjhb = switchBlock.OutSignal()->Read();





//    SandboxMassSpringDamper();
//    SandboxEngine();
//    SandboxTransmission();
//    SandboxVehicleStandalone();
//    SandboxFullModel();

    Models::LockupClutch clutch;

    clutch.SetGearRatio(5);



    return 0;
}
