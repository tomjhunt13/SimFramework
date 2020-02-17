#include <iostream>
#include <string>
#include <vector>

#include "Framework.h"
#include "Components.h"

#include "Mass1D.h"
#include "SpringDamper1D.h"
#include "OutputBlock.h"



int main() {

    SimFramework::Signal<Eigen::Vector2f> signal1; // ConstantBlock States
    SimFramework::Signal<Eigen::Vector2f> signal2; // Mass1D States
    SimFramework::Signal<float> signal3; // SpringDamper Force

    // Create blocks
    SimFramework::ConstantBlock<Eigen::Vector2f> constBlock (&signal1, {0.f, 0.f});
    SpringDamper1D spring(&signal1, &signal2, &signal3);
    Mass1D mass (&signal3, &signal2, {0.1, 0.f});
    OutputBlock out(&signal1, &signal3);

    // Construct system
    SimFramework::SystemManager& systemManager = SimFramework::SystemManager::Get();
    systemManager.RegisterBlocks({&constBlock, &mass, &spring, &out});


    // Iterate
    for (float t = 0; t <= 5; t += 0.01) {
        systemManager.UpdateSystem(t);
    }

    return 0;
}
