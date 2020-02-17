#ifndef SIMINTERFACE_SPRINGDAMPER1D_H
#define SIMINTERFACE_SPRINGDAMPER1D_H

#include "Eigen/Dense"
#include "Framework.h"

class SpringDamper1D : public SimFramework::Block {

public:

    SpringDamper1D(SimFramework::Signal<Eigen::Vector2f>* inputConnection1,
                   SimFramework::Signal<Eigen::Vector2f>* inputConnection2,
                   SimFramework::Signal<float>* outputForce);

    // Block functions
    void Read() override;
    void Write() override;
    void Update(float t_np1) override;

private:

    // Signals
    SimFramework::Signal<Eigen::Vector2f>* m_InputConnection1;
    SimFramework::Signal<Eigen::Vector2f>* m_InputConnection2;
    SimFramework::Signal<float>* m_OutputForce;

    // Copies
    Eigen::Vector2f m_In1Copy;
    Eigen::Vector2f m_In2Copy;
    float m_OutCopy;

    // Physical Properties
    float k = 1.f;
    float c = 0.f;

};


#endif //SIMINTERFACE_SPRINGDAMPER1D_H
