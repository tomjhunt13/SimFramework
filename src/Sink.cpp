#include "Sink.h"

void Sink::Update(float time) {

    std::cout << "t: " << time << ", x:  " <<  this->m_InputSignal.Read() << std::endl;

}
