#ifndef SIMINTERFACE_RK4_H
#define SIMINTERFACE_RK4_H

#include "DynamicSystem.h"

//namespace SimInterface {
//
//    class RK4 {
//
//    public:
//
//        template <typename Type>
//        static Type Step(DynamicSystem &system, float dt, float t, Type &x);
//    };
//
//    template <typename Type>
//    Type RK4::Step(DynamicSystem &system, float dt, float t, Type &x) {
//
//        Type k1 = system.Gradient(t, x);
//        Type k2 = system.Gradient(t + (dt / 2.f), x + k1 / 2.f);
//        Type k3 = system.Gradient(t + (dt / 2.f), x + k2 / 2.f);
//        Type k4 = system.Gradient(t + dt, x + k3);
//
//        return x + (1.f / 6.f) * (k1 + 2.f * k2 + 2.f * k3 + k4);
//    }
//
//} // namespace SimInterface


#endif //SIMINTERFACE_RK4_H
