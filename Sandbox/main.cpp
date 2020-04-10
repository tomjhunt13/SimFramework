//#include "SandboxMassSpringDamper.h"
//#include "SandboxEngine.h"
//#include "SandboxFullModel.h"

#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"
#include "SimModels/LockupClutch.h"

int main() {
    SimFramework::Input<float> in;
    Models::CrossingDetect cross;
    SimFramework::Output<bool> out;

    in.Configure(1.f);
    cross.Configure(in.OutSignal());
    cross.SetParameters(3.f, false);
    out.Configure(cross.OutCrossing(), false);


    in.Initialise(0.f);

    in.Update(0.f);
    cross.Update();
    out.Update(0.f);

    bool value = out.ReadValue();

    in.Update(0.f);
    cross.Update();
    out.Update(0.f);

    value = out.ReadValue();

    in.WriteValue(4.f);
    in.Update(0.f);
    cross.Update();
    out.Update(0.f);

    value = out.ReadValue();

    in.Update(0.f);
    cross.Update();
    out.Update(0.f);

    value = out.ReadValue();

    in.Update(0.f);
    cross.Update();
    out.Update(0.f);

    value = out.ReadValue();

    in.Update(0.f);
    cross.Update();
    out.Update(0.f);

    value = out.ReadValue();

    in.WriteValue(8.f);
    in.Update(0.f);
    cross.Update();
    out.Update(0.f);

    value = out.ReadValue();

    in.Update(0.f);
    cross.Update();
    out.Update(0.f);

    value = out.ReadValue();

    in.WriteValue(2.f);
    in.Update(0.f);
    cross.Update();
    out.Update(0.f);

    value = out.ReadValue();

    in.Update(0.f);
    cross.Update();
    out.Update(0.f);

    value = out.ReadValue();

    in.Update(0.f);
    cross.Update();
    out.Update(0.f);

    value = out.ReadValue();








//    SandboxMassSpringDamper();
//    SandboxEngine();
//    SandboxTransmission();
//    SandboxVehicleStandalone();
//    SandboxFullModel();

    Models::LockupClutch clutch;

    clutch.SetGearRatio(5);



    return 0;
}
