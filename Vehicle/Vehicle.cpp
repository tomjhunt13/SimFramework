#include "Vehicle.h"

#include <iostream>
#include <string>


#include "Framework.h"

struct testStruct
{
    float a = 2;
    std::string b = "testString";
};

namespace Vehicle {


    struct VehicleComponents
    {
        // Signals
        SimFramework::Signal<float>* ThrottlePosition;
        SimFramework::Signal<float>* EngineSpeed;
        SimFramework::Signal<float>* EngineTorque;

        // Engine
//        Vehicle::En


        testStruct ts;
    };

    VehicleC::VehicleC()
    {
        this->components = new VehicleComponents;
    }

    VehicleC::~VehicleC()
    {
        delete this->components;
    }

    void VehicleC::Print() {
        std::cout << this->components->ts.a << this->components->ts.b << std::endl;
    }

}