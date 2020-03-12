#include "SimFramework/Components.h"

namespace SimFramework {

    void LookupTable2D::Configure(Signal<float>* x, Signal<float>* y, Signal<float>* out)
    {
        this->m_X = x;
        this->m_Y = y;
        this->m_Out = out;
    }

    void LookupTable2D::SetTable(Table3D &table)
    {
        this->m_Table = table;
    }

    std::vector<SignalBase*> LookupTable2D::InputSignals()
    {
        return {this->m_X, this->m_Y};
    }

    std::vector<SignalBase*> LookupTable2D::OutputSignals()
    {
        return {this->m_Out};
    }

    void LookupTable2D::Update()
    {
        float x = this->m_X->Read();
        float y = this->m_Y->Read();

        this->m_Out->Write(InterpTable3D(this->m_Table, {x, y}));
    };


//
//
//    void TriggerFunction::Configure(Signal<float>* outputSignal)
//    {
//        this->m_SOutput = outputSignal;
//
//    };
//
//    void TriggerFunction::Trigger()
//    {
//        this->m_State = true;
//        this->t_n = 0.f;
//    };
//
//    void TriggerFunction::Read() {};
//
//    void TriggerFunction::Write()
//    {
//        this->m_SOutput->Write(this->m_OutCopy);
//    };
//
//    void TriggerFunction::Update(float dt)
//    {
//
//        if (this->m_State)
//        {
//            this->m_OutCopy = this->Evaluate(this->t_n);
//
//            this->t_n += dt;
//
//            if (this->t_n > this->t_end)
//            {
//                this->m_State = false;
//            }
//        }
//
//        else
//        {
//            this->m_OutCopy = this->m_Default;
//        }
//    };
//
//    void TriggerFunction::Init(float t_0)
//    {
//        this->m_State = false;
//        this->m_OutCopy = this->m_Default;
//    };

}; // namespace Framework