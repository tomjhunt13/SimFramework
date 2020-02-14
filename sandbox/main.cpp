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






    SimFramework::SystemManager& sm = SimFramework::SystemManager::Get();
    sm.ConstructSystem();
    sm.UpdateSystem(3);


    return 0;
}
