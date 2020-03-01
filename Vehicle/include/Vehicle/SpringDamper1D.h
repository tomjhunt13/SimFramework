#ifndef SIMINTERFACE_SPRINGDAMPER1D_H
#define SIMINTERFACE_SPRINGDAMPER1D_H

#include "Eigen/Dense"
#include "SimFramework/Framework.h"

namespace Vehicle {

    class SpringDamper1D : public SimFramework::Block {

    public:

        SpringDamper1D(SimFramework::Signal<Eigen::Vector2f>* inputConnection1,
                       SimFramework::Signal<Eigen::Vector2f>* inputConnection2,
                       SimFramework::Signal<float>* outputForce);

        // Block functions
        void Read() override;

        void Write() override;

        void Update(float t_np1) override;

        void Init(float t_0) override;

    private:

        // Signals
        SimFramework::Signal<Eigen::Vector2f> *m_InputConnection1;
        SimFramework::Signal<Eigen::Vector2f> *m_InputConnection2;
        SimFramework::Signal<float> *m_OutputForce;

        // Copies
        Eigen::Vector2f m_In1Copy;
        Eigen::Vector2f m_In2Copy;
        float m_OutCopy;

        // Physical Properties
        float k = 8.f;
        float c = 1.f;

    };

} // namespace Vehicle

#endif //SIMINTERFACE_SPRINGDAMPER1D_H
