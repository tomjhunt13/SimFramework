//
// Created by Thomas Hunt on 05/02/2020.
//

#ifndef SIMINTERFACE_SIGNAL_H
#define SIMINTERFACE_SIGNAL_H


// Parent Signal class to reference generic template class
class Signal {};


// Templated signal to allow different signal types
template <typename type>
class SignalTyped : public Signal {

private:
    type m_Value;

public:
    SignalTyped();

    const type Read() const { return this->m_Value; };

    void Write(type& value) { this->m_Value = value; };
};


#endif //SIMINTERFACE_SIGNAL_H
