#include <iostream>

#include "SimFramework/Components.h"


#include "SandboxMassSpringDamper.h"
#include "SandboxEngine.h"
#include "SandboxTransmission.h"
#include "SandboxVehicleStandalone.h"
#include "SandboxFullModel.h"


int main() {

    SimFramework::Input<float> cnst;
    cnst.Configure(2.4);
    cnst.Initialise(0.f);

    cnst.WriteValue(4.f);
    cnst.Update(0.f);


    SimFramework::Gain<float, float> g;

    const SimFramework::Signal<float>* out = g.OutSignal();
    SimFramework::Signal<float> temp;

    g.Configure(cnst.OutSignal(), &temp, 3.f);

    g.Update();

    std::cout << out->Read() << std::endl;


//    SandboxFullModel();
//    SandboxMassSpringDamper();
//    SandboxEngine();
//    SandboxTransmission();
//    SandboxVehicleStandalone();

    return 0;
}
