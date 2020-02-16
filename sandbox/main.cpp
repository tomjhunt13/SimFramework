#include <iostream>
#include <string>
#include <vector>

#include "Framework.h"
#include "Components.h"

#include "Mass1D.h"
#include "SpringDamper1D.h"
#include "OutputBlock.h"


int main() {

    SimFramework::Signal signal1; // ConstantBlock States
    SimFramework::Signal signal2; // Mass1D States
    SimFramework::Signal signal3; // SpringDamper Force

    // Create blocks
    Eigen::VectorXf a (2, 1);
    a(0) = 0.f;
    a(1) = 0.f;
    SimFramework::ConstantBlock cnstblk (&signal1, a);
    SpringDamper1D sd(&signal1, &signal2, &signal3);
    Mass1D mass (&signal3, &signal2);
    OutputBlock out(&signal2, &signal3);




    // Test system
    SimFramework::SystemManager& systemManager = SimFramework::SystemManager::Get();
    for (float t = 0; t <= 5; t += 0.01) {

//        signal2.Write({1, 2});

        systemManager.UpdateSystem(t);
    }


    return 0;
}
