//#include "SandboxMassSpringDamper.h"
//#include "SandboxEngine.h"
//#include "SandboxTransmission.h"

#include <vector>
#include "SimFramework/Framework.h"
#include "SimFramework/Components.h"

int main() {

    SimFramework::Signal<float> a1;
    SimFramework::Signal<float> a2;
    SimFramework::Signal<float> a3;
    SimFramework::Signal<float> b1;

    SimFramework::Signal<float>* p1 = &a1;
    SimFramework::Signal<float>* p2 = &a2;
    SimFramework::Signal<float>* p3 = &a3;



    SimFramework::SummingJunction<float> b;
    b.Configure({p1, p2, p3}, &b1, {1, 1, 1});

    std::vector<SimFramework::SignalBase*> s1 = b.InputSignals();
    std::vector<SimFramework::SignalBase*> s2 = b.OutputSignals();


//    SandboxMassSpringDamper();
//    SandboxEngine();
//    SandboxTransmission();
    return 0;
}
