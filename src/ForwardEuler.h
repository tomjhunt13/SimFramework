#ifndef SIMINTERFACE_FORWARDEULER_H
#define SIMINTERFACE_FORWARDEULER_H

#include "DynamicSystem.h"

namespace SimInterface {

    class ForwardEuler {
    public:
        template<typename GradientType>
        static GradientType Step(DynamicSystem<GradientType> &system, float dt, float t, GradientType &x_n);
    };


    template <typename GradientType>
    GradientType ForwardEuler::Step(DynamicSystem<GradientType> &system, float dt, float t_n, GradientType &x_n)
    {
        return x_n + dt * system.Gradient(t_n, x_n);
    }

}; // namespace SimInterface


#endif //SIMINTERFACE_FORWARDEULER_H
