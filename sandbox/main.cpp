#include <iostream>
#include <string>
#include <vector>

#include "Framework.h"

class pfunc : public SimFramework::Function
{
public:
    pfunc(std::vector<SimFramework::Signal*> inputSignals, SimFramework::Signal* outputSignal, std::string name) : m_Name(name)
    {
        for (SimFramework::Signal* input : inputSignals)
        {
            this->RegisterInputSignal(*input);
        }

        this->RegisterOutputSignal(*outputSignal);
    };

    // Block API
    void Read() override {};

    void Update(float t_np1) override
    {
        std::cout << this->m_Name << std::endl;
    };

    void Write() override {};

private:
    std::string m_Name;
};


int main() {

    // Unconnected blocks
//    pfunc q1("a");
//
//    // First graph
//    pfunc p2("2");
//    pfunc p7("7");
//    pfunc p3("3");
//    pfunc q3("c");
//    pfunc p4("4");
//    pfunc p5("5");
//    pfunc q2("b");
//    pfunc p6("6");
//    pfunc p1("1");
//
//
//    SimFramework::Signal s13(&p1);
//    SimFramework::Signal s23(&p2);
//    SimFramework::Signal s34(&p3);
//    SimFramework::Signal s45(&p4);
//    SimFramework::Signal s65(&p6);
//    SimFramework::Signal s57(&p5);


    SimFramework::Signal s1;
    SimFramework::Signal s2;
    SimFramework::Signal s3;
    SimFramework::Signal s4;
    SimFramework::Signal s5;
    SimFramework::Signal s6;
    SimFramework::Signal s7;
    SimFramework::Signal s8;
    SimFramework::Signal s9;
    SimFramework::Signal s10;
    SimFramework::Signal s11;

    // Second graph
    pfunc r1({}, &s1, "r1");
    pfunc r2({}, &s2, "r2");
    pfunc r3({&s1}, &s3, "r3");
    pfunc r4({&s1, &s2}, &s4, "r4");
    pfunc r5({&s3}, &s5, "r5");
    pfunc r6({&s4}, &s6, "r6");
    pfunc r7({&s5, &s6}, &s7, "r7");
    pfunc r8({}, &s8, "r8");
    pfunc r9({}, &s9, "r9");
    pfunc r10({&s9}, &s10, "r10");
    pfunc r11({&s7, &s8, &s10}, &s11, "r11");




    SimFramework::SystemManager& sm = SimFramework::SystemManager::Get();
    sm.ConstructSystem();
    sm.UpdateSystem(3);


    return 0;
}
