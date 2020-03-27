#include "SandboxMassSpringDamper.h"
#include "SandboxEngine.h"
#include "SandboxTransmission.h"
#include "SandboxVehicleStandalone.h"
#include "SandboxFullModel.h"

#include "SimFramework/Framework.h"



int main() {



    Eigen::Vector<float, 3> vec;
    vec << 0.3, 0.4, -0.1;

    SimFramework::Signal<Eigen::Vector<float, 3>> sig;
//    SimFramework::Signal<float> sig;
//    sig.Write(3.34);
    sig.Write(vec);


    float f = 3.2;
    bool a = true;

    std::cout << sig.Value() << std::endl;
    std::cout << SimFramework::ToString(f) << std::endl;
    std::cout << SimFramework::ToString(a) << std::endl;






//    SandboxMassSpringDamper();
//    SandboxEngine();
//    SandboxTransmission();
//    SandboxVehicleStandalone();
//    SandboxFullModel();


    return 0;
}
