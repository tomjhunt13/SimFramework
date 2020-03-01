#ifndef SIMINTERFACE_OUTPUTBLOCK_H
#define SIMINTERFACE_OUTPUTBLOCK_H

#include <iostream>

#include "Eigen/Dense"
#include "SimFramework/Framework.h"



class OutputBlock : public SimFramework::Block {

public:
    OutputBlock(SimFramework::Signal<Eigen::Vector2f>* massStates, SimFramework::Signal<float>* force);

    // Block functions
    void Read() override;
    void Write() override {};
    void Update(float t_np1) override;
    void Init(float t_0) override;

private:

    // Signals
    SimFramework::Signal<Eigen::Vector2f>* m_MassStates;
    SimFramework::Signal<float>* m_Force;

    // Copies
    Eigen::Vector2f m_StatesCopy;
    float m_ForceCopy;


    float t_n = 0;
};


#endif //SIMINTERFACE_OUTPUTBLOCK_H
