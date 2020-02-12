#ifndef SIMINTERFACE_SIGNAL_H
#define SIMINTERFACE_SIGNAL_H


namespace SimFramework {

// Templated signal to allow different signal types
    template<typename type>
    class Signal {

    private:
        type m_Value;

    public:
        Signal(type initialValue) : m_Value(initialValue) {};

        const type Read() const { return this->m_Value; };


        // TODO: Dont want to instantiate new vector each time
        void Write(type value) { this->m_Value = value; };
    };

} // namespace SimFramework


#endif //SIMINTERFACE_SIGNAL_H
