#ifndef SIMINTERFACE_DYNAMICSYSTEM_H
#define SIMINTERFACE_DYNAMICSYSTEM_H


namespace SimFramework {

    template <typename GradientType>
    class DynamicSystem {
    public:
        virtual GradientType Gradient(float t, GradientType x) = 0;
    };

}

#endif //SIMINTERFACE_DYNAMICSYSTEM_H
