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

    // Block API
    void LookupTable2D::Read()
    {
        this->m_XCopy = this->m_X->Read();
        this->m_YCopy = this->m_Y->Read();
    };

    void LookupTable2D::Write()
    {
        this->m_Out->Write(m_OutCopy);
    };

    void LookupTable2D::Update(float dt)
    {
        this->m_OutCopy = InterpTable3D(this->m_Table, {this->m_XCopy, this->m_YCopy});
    };

    void LookupTable2D::Init(float t_0) {};

}; // namespace Framework