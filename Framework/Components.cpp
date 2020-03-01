#include "SimFramework/Components.h"

namespace SimFramework {

    LookupTable2D::LookupTable2D(Table3D& table, Signal<float>* x, Signal<float>* y, Signal<float>* out) :
                                    m_Table(table), m_X(x), m_Y(y), m_Out(out) {};

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

    void LookupTable2D::Update(float t_np1)
    {
        this->m_OutCopy = InterpTable3D(this->m_Table, {this->m_XCopy, this->m_YCopy});
    };

    void LookupTable2D::Init(float t_0) {};

}; // namespace Framework