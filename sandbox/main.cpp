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


class Transmission : public SimFramework::Model
{

public:
    Transmission()
    {
        this->blend.Configure(&(this->val1Sig), &(this->val2Sig), &(this->trigSig), &(this->outSig));
        this->trig.Configure(&(this->trigSig));
        this->output.Configure(&(this->outSig), 0.f);

        // Set values
        val1Sig.Write(1);
        val2Sig.Write(2);

        this->RegisterBlocks({}, {}, {&(this->trig), &(this->blend)}, {&(this->output)});
    };

    SimFramework::Output<float>* OutputBlock()
    {
        return &(this->output);
    };

    LinearTrigger* TriggerBlock()
    {
        return &(this->trig);
    };


private:

    // Signals
    SimFramework::Signal<float> val1Sig;
    SimFramework::Signal<float> val2Sig;
    SimFramework::Signal<float> trigSig;
    SimFramework::Signal<float> outSig;

    // Blocks
    SimFramework::LinearBlend<float> blend;
    LinearTrigger trig;
    SimFramework::Output<float>  output;
};

int main() {


    Transmission transmission;
    LinearTrigger* trig = transmission.TriggerBlock();
    SimFramework::Output<float>* outBlock = transmission.OutputBlock();
    transmission.Initialise(0);

    std::ofstream myfile;
    myfile.open ("tmpOut.csv", std::ios::out);


    float dt = 0.05;
    int counter = 0;
    for (float t = 0.f; t <= 80.f; t += dt) {

        if (counter % 100 == 0)
        {
            trig->Trigger();
        }

        transmission.Update(t);

        myfile << t << ", " << outBlock->ReadValue() << std::endl;
        std::cout << "t: " << t << ", val: " << outBlock->ReadValue() << std::endl;

        counter ++;
    }

    myfile.close();

    return 0;
}
