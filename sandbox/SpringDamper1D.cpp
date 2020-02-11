#include "SpringDamper1D.h"

#include <iostream>

SpringDamper1D::SpringDamper1D(
        SimInterface::Signal<Eigen::Vector2f>& inputConnection1,
        SimInterface::Signal<Eigen::Vector2f>& inputConnection2,
        SimInterface::Signal<float>& outputForce)
        : inputConnection1(&inputConnection1), inputConnection2(&inputConnection2), outputForce(&outputForce)
{
    this->Write();
}


void SpringDamper1D::Read()
{
    this->connection1 = this->inputConnection1->Read();
    this->connection2 = this->inputConnection2->Read();

    std::cout << this->connection2 << std::endl;

};

void SpringDamper1D::Write()
{
    this->outputForce->Write(this->force);
};

void SpringDamper1D::Update(float t)
{
    this->force = this->k * (this->connection2[0] - this->connection1[0]) + this->c * (connection2[1] - connection1[1]);
};