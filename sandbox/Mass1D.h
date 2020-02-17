#ifndef SIMINTERFACE_MASS1D_H
#define SIMINTERFACE_MASS1D_H

#include "Eigen/Dense"
#include "../src/Framework.h"



class Mass1D : public SimFramework::Block, public SimFramework::DynamicSystem<Eigen::Vector2f> {

public:

    Mass1D(SimFramework::Signal<float>* inputSpringForce, SimFramework::Signal<Eigen::Vector2f>* outputStates, Eigen::Vector2f initialStates);

    // Block functions
    void Read() override;
    void Write() override;
    void Update(float t_np1) override;

    // Dynamic system functions
    Eigen::Vector2f Gradient(float t, Eigen::Vector2f x) override;

private:

    // Signals
    SimFramework::Signal<float>* m_InputSpringForce;
    SimFramework::Signal<Eigen::Vector2f>* m_OutputStates;

    // Internal copies
    float m_InputCopy;
    Eigen::Vector2f m_States;
    float t_n = 0;

    // Physical Properties
    float mass = 1;
};


#endif //SIMINTERFACE_MASS1D_H
