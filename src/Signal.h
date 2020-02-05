//
// Created by Thomas Hunt on 05/02/2020.
//

#ifndef SIMINTERFACE_SIGNAL_H
#define SIMINTERFACE_SIGNAL_H

/*
// Parent Signal class to reference generic template class
class Signal {

public:

    virtual void Write();

//    void Test(something & output) {
//        something = this->Read();
//    }
};
*/


// Templated signal to allow different signal types
template <typename type>
class Signal {

private:
    type m_Value;

public:
    Signal(type initialValue) : m_Value(initialValue) {};

    const type Read() const { return this->m_Value; };


    // TODO: Dont want to instantiate new vector each time
    void Write(type& value) { this->m_Value = value; };
};


#endif //SIMINTERFACE_SIGNAL_H
