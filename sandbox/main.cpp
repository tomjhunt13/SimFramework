#include <iostream>
#include <string>

#include "Framework.h"

class pfunc : public SimFramework::Block
{
public:
    pfunc(std::string name) : SimFramework::Block(SimFramework::Function), m_Name(name) {};

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
    pfunc q1("a");




    // First graph
    pfunc p2("2");
    pfunc p7("7");
    pfunc p3("3");
    pfunc q3("c");
    pfunc p4("4");
    pfunc p5("5");
    pfunc q2("b");
    pfunc p6("6");
    pfunc p1("1");


    SimFramework::Signal s13(&p1);
    SimFramework::Signal s23(&p2);
    SimFramework::Signal s34(&p3);
    SimFramework::Signal s45(&p4);
    SimFramework::Signal s65(&p6);
    SimFramework::Signal s57(&p5);

    p3.RegisterInputSignal(s13);
    p3.RegisterInputSignal(s23);
    p4.RegisterInputSignal(s34);
    p5.RegisterInputSignal(s45);
    p5.RegisterInputSignal(s65);
    p7.RegisterInputSignal(s57);


    // Second graph
    pfunc r1("r1");
    pfunc r2("r2");
    pfunc r3("r3");
    pfunc r4("r4");
    pfunc r5("r5");
    pfunc r6("r6");
    pfunc r7("r7");
    pfunc r8("r8");
    pfunc r9("r9");
    pfunc r10("r10");
    pfunc r11("r11");

    SimFramework::Signal t13(&r1);
    SimFramework::Signal t14(&r1);
    SimFramework::Signal t24(&r2);
    SimFramework::Signal t35(&r3);
    SimFramework::Signal t46(&r4);
    SimFramework::Signal t57(&r5);
    SimFramework::Signal t67(&r6);
    SimFramework::Signal t711(&r7);
    SimFramework::Signal t811(&r8);
    SimFramework::Signal t910(&r9);
    SimFramework::Signal t1011(&r10);

    r3.RegisterInputSignal(t13);
    r4.RegisterInputSignal(t14);
    r4.RegisterInputSignal(t24);
    r5.RegisterInputSignal(t35);
    r6.RegisterInputSignal(t46);
    r7.RegisterInputSignal(t57);
    r7.RegisterInputSignal(t67);
    r11.RegisterInputSignal(t711);
    r11.RegisterInputSignal(t811);
    r11.RegisterInputSignal(t1011);
    r10.RegisterInputSignal(t910);





    SimFramework::SystemManager& sm = SimFramework::SystemManager::Get();
    sm.ConstructSystem();
    sm.UpdateSystem(3);


    return 0;
}
