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
        // Engine


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