#include "SimFramework/Components.h"

namespace SimFramework {

    TriggerFunction::TriggerFunction(std::string name) : Source(name) {};

    void TriggerFunction::Trigger()
    {
        this->m_State = true;
        this->t_n = 0.f;
    };

    const Signal<float>* TriggerFunction::OutSignal() const
    {
        return &(this->m_Output);
    };


    std::vector<SignalBase*> TriggerFunction::InputSignals()
    {
        return {};
    }

    std::vector<SignalBase*> TriggerFunction::OutputSignals()
    {
        return {&(this->m_Output)};
    }

    void TriggerFunction::Initialise(float t_0)
    {
        this->m_State = false;
        this->m_Output.Write(this->m_Default);
    };

    void TriggerFunction::Update(float dt)
    {
        float output;

        this->t_n += dt;

        if (this->t_n > this->t_end)
        {
            this->m_State = false;
        }

        if (this->m_State)
        {
            output = this->Evaluate(this->t_n);
        }

        else
        {
            output = this->m_Default;
        }

        this->m_Output.Write(output);
    };


    LookupTable2D::LookupTable2D(std::string name) : Function(name) {};

    void LookupTable2D::Configure(const Signal<float>* x, const Signal<float>* y)
    {
        this->m_X = x;
        this->m_Y = y;
    }

    const Signal<float>* LookupTable2D::OutSignal() const
    {
        return  &(this->m_Out);
    };

    void LookupTable2D::SetTable(Table3D &table)
    {
        this->m_Table = table;
    }

    std::vector<SignalBase*> LookupTable2D::InputSignals()
    {
        return {}; // this->m_X, this->m_Y};
    }

    std::vector<SignalBase*> LookupTable2D::OutputSignals()
    {
        return {&(this->m_Out)};
    }

    void LookupTable2D::Update()
    {
        float x = this->m_X->Read();
        float y = this->m_Y->Read();

        this->m_Out.Write(InterpTable3D(this->m_Table, {x, y}));
    };

}; // namespace Framework