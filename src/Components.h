#ifndef SIMINTERFACE_CONSTANTBLOCK_H
#define SIMINTERFACE_CONSTANTBLOCK_H

#include "Eigen/Dense"
#include "Framework.h"

namespace SimFramework {

    //-------------------- Constant
    class ConstantBlock : public Source {

    public:
        ConstantBlock(Signal* outputSignal, Eigen::VectorXf value);
        void Update(float t_np1) override { this->t_n = t_np1; }

    private:
        Eigen::VectorXf m_Value;
    };


    //------------------------ Summing Junction
    class SummingJunction : public Function {
    public:
        SummingJunction(std::vector<Signal*> inputSignals,
                        Signal* outputSignal,
                        std::vector<float> weights);

        void Update(float t) override;

    private:

        // Parameters
        std::vector<float> m_Weights;
        int m_nInputs;

    };

}; // namespace SimFramework


#endif //SIMINTERFACE_CONSTANTBLOCK_H
