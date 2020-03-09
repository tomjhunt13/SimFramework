#include <iostream>
#include <fstream>
#include <string>

#include "SimFramework/Components.h"

class LinearTrigger : public SimFramework::TriggerFunction
{
public:
    LinearTrigger()
    {
        this->m_Default = 0.f;
        this->t_end = 2.f;
    }

    float Evaluate(float t)
    {
        return 1.f * (this->t_end - t) / (this->t_end);
    }
};

int main() {

    SimFramework::Signal<float> sig;
    LinearTrigger trig;
    trig.Configure(&sig);


    std::ofstream myfile;
    myfile.open ("tmpOut.csv", std::ios::out);


    float dt = 0.05;
    int counter = 0;
    for (float t = 0.f; t <= 80.f; t += dt) {

        if (counter % 100 == 0)
        {
            trig.Trigger();
        }

        trig.Read();
        trig.Update(dt);
        trig.Write();




        myfile << t << ", " << sig.Read() << std::endl;
        std::cout << "t: " << t << ", alpha: " << sig.Read() << std::endl;

        counter ++;
    }

    myfile.close();

    return 0;
}
