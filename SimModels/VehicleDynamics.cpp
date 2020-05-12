#include "SimModels/VehicleDynamics.h"


namespace Models {

    AeroDrag::AeroDrag(std::string name) : Function(name) {};

    void AeroDrag::SetParameters(float Cd, float A, float rho)
    {
        this->Cd = Cd;
        this->A = A;
        this->rho = rho;
    }

    void AeroDrag::Configure(const SimFramework::Signal<float>* inSpeed)
    {
        this->m_Speed = inSpeed;
    };

    const SimFramework::Signal<float>* AeroDrag::OutForce() const
    {
        return &(this->m_Force);
    };


    std::vector<const  SimFramework::SignalBase*> AeroDrag::InputSignals() const
    {
        return {this->m_Speed};
    };

    std::vector<const SimFramework::SignalBase*> AeroDrag::OutputSignals() const
    {
        return {&(this->m_Force)};
    };

    void AeroDrag::Update()
    {
        float speed = this->m_Speed->Read();

        // Calculate output force
        float force = 0.5 * this->rho * this->A * this->Cd * speed * speed;

        // Determine directionality
        if (speed < 0)
        {
            force *= -1.f;
        };

        // Write output
        this->m_Force.Write(force);
    };


    Gravity::Gravity(std::string name) : Function(name) {};

    void Gravity::SetParameters(float mass, float g)
    {
        this->mass = mass;
        this->g = g;
    };

    void Gravity::Configure(const SimFramework::Signal<float>* inGradient)
    {
        this->m_Gradient = inGradient;
    };

    const SimFramework::Signal<float>* Gravity::OutForce() const
    {
        return &(this->m_Force);
    };

    std::vector<const SimFramework::SignalBase*> Gravity::InputSignals() const
    {
        return {this->m_Gradient};
    };

    std::vector<const SimFramework::SignalBase*> Gravity::OutputSignals() const
    {
        return {this->OutForce()};
    };

    void Gravity::Update()
    {
        float gradient = this->m_Gradient->Read();
        this->m_Force.Write(this->mass * this->g * std::sin(gradient));
    };


    VehicleDynamics::VehicleDynamics() : m_AeroDrag("Drag"), m_Gravity("Gravity"), m_Vectorise("Vehicle Force Input"), m_StateSpace("Vehicle Dynamics"), m_Mask("Vehicle Dynamics") {};

    void VehicleDynamics::SetParameters(float initialPosition, float initialVelocity, float mass, float Cd, float A, float rho, float rollingResistance, float g) {

        this->m_AeroDrag.SetParameters(Cd, A, rho);
        this->m_Gravity.SetParameters(mass, g);
        this->m_RollingResistance.SetParameters(rollingResistance);
        this->m_ConstWeight.Configure(mass * g);

        // Set up state matrices
        Eigen::Matrix<float, 2, 2> matA;
        matA << 0.f, 1.f, 0.f, 0.f;

        Eigen::Matrix<float, 2, 4> B;
        B << 0, 0, 0, 0, 1.f / mass, -1.f / mass, -1.f / mass, -1.f / mass;

        Eigen::Matrix<float, 2, 2> C;
        C << 1.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<float, 2, 4> D;
        D << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;

        this->m_StateSpace.SetMatrices(matA, B, C, D);
        Eigen::Vector2f initialConditions = {initialPosition, initialVelocity};
        this->m_StateSpace.SetInitialConditions(initialConditions);
    };


    void VehicleDynamics::Configure(
            const SimFramework::Signal<float>* inTyreForce,
            const SimFramework::Signal<float>* inGradient)
    {
        // Configure blocks
        this->m_AeroDrag.Configure(this->OutVehicleVelocity());
        this->m_Gravity.Configure(inGradient);
        this->m_RollingResistance.Configure(this->OutVehicleVelocity(), this->m_ConstWeight.OutSignal());
        this->m_Vectorise.Configure({inTyreForce, this->m_AeroDrag.OutForce(), this->m_Gravity.OutForce(), this->m_RollingResistance.OutForce()});
        this->m_StateSpace.Configure(this->m_Vectorise.OutSignal());
        this->m_Mask.Configure(this->m_StateSpace.OutSignal());
    };

    const SimFramework::Signal<float>* VehicleDynamics::OutVehiclePosition() const
    {
        return this->m_Mask.OutSignal(0);
    };

    const SimFramework::Signal<float>* VehicleDynamics::OutVehicleVelocity() const
    {
        return this->m_Mask.OutSignal(1);
    };

    SimFramework::BlockList VehicleDynamics::Blocks()
    {
        return {{&(this->m_ConstWeight)},
                {&(this->m_StateSpace)},
                {&(this->m_AeroDrag), &(this->m_Gravity), &(this->m_Vectorise), &(this->m_Mask), &(this->m_RollingResistance)},
                {},
                {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > VehicleDynamics::LogSignals()
    {
        return {{"Vehicle Position, Vehicle Velocity", this->m_StateSpace.OutSignal()},
                {"Vehicle Tyre Force, Vehicle Aero Drag, Vehicle Gravity Force, Vehicle RollingResistance", this->m_Vectorise.OutSignal()}};
    };

}; // namespace Models