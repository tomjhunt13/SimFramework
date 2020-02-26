#include "SpringDamper1D.h"


namespace Vehicle {
    SpringDamper1D::SpringDamper1D(
            SimFramework::Signal<Eigen::Vector2f> *inputConnection1,
            SimFramework::Signal<Eigen::Vector2f> *inputConnection2,
            SimFramework::Signal<float> *outputForce)
            : m_InputConnection1(inputConnection1), m_InputConnection2(inputConnection2), m_OutputForce(outputForce) {};


    void SpringDamper1D::Read() {
        this->m_In1Copy = this->m_InputConnection1->Read();
        this->m_In2Copy = this->m_InputConnection2->Read();
    };

    void SpringDamper1D::Write() {
        this->m_OutputForce->Write(this->m_OutCopy);
    };

    void SpringDamper1D::Update(float t_np1) {
        this->m_OutCopy = this->k * (this->m_In1Copy[0] - this->m_In2Copy[0]) +
                          this->c * (this->m_In1Copy[1] - this->m_In2Copy[1]);
    };

    void SpringDamper1D::Init(float t_0) {
//    this->m_OutputForce->Write(0);
    }

}; // namespace Vehicle