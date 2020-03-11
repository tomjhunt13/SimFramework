#include "SimFramework/Components.h"

namespace SimFramework {

//    void LookupTable2D::Configure(Signal<float>* x, Signal<float>* y, Signal<float>* out)
//    {
//        this->m_X = x;
//        this->m_Y = y;
//        this->m_Out = out;
//    }
//
//    void LookupTable2D::SetTable(Table3D &table)
//    {
//        this->m_Table = table;
//    }
//
//    // Block API
//    void LookupTable2D::Read()
//    {
//        this->m_XCopy = this->m_X->Read();
//        this->m_YCopy = this->m_Y->Read();
//    };
//
//    void LookupTable2D::Write()
//    {
//        this->m_Out->Write(m_OutCopy);
//    };
//
//    void LookupTable2D::Update(float dt)
//    {
//        this->m_OutCopy = InterpTable3D(this->m_Table, {this->m_XCopy, this->m_YCopy});
//    };
//
//    void LookupTable2D::Init(float t_0) {};
//
//
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