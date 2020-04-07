#include <iostream> // TODO: remove

#include "SimModels/VehicleComponents.h"


namespace Models {

    Clutch::Clutch(std::string name) : Function(name) {};

    void Clutch::Configure(
            const SimFramework::Signal<float>* inEngineSpeed,
            const SimFramework::Signal<float>* inTransmissionSpeed,
            const SimFramework::Signal<float>* inClutchStiffness)
    {
        this->m_InEngineSpeed = inEngineSpeed;
        this->m_InTransmissionSpeed = inTransmissionSpeed;
        this->m_InClutchStiffness = inClutchStiffness;
    };

    const SimFramework::Signal<float>* Clutch::OutClutchTorque() const
    {
        return &(this->m_OutClutchTorque);
    };


    std::vector<const SimFramework::SignalBase*> Clutch::InputSignals() const
    {
        return {this->m_InEngineSpeed, this->m_InTransmissionSpeed, this->m_InClutchStiffness};
    };

    std::vector<const SimFramework::SignalBase*> Clutch::OutputSignals() const
    {
        return {&(this->m_OutClutchTorque)};
    };

    void Clutch::Update()
    {
        this->m_OutClutchTorque.Write(this->m_InClutchStiffness->Read() * (this->m_InEngineSpeed->Read() - this->m_InTransmissionSpeed->Read()));
    };


    LinearTrigger::LinearTrigger(std::string name) : SimFramework::TriggerFunction(name) {};

    void LinearTrigger::SetParameters(float defaultValue, float t_end)
    {
        this->m_Default = defaultValue;
        this->t_end = 1.f;
    }

    float LinearTrigger::Evaluate(float t) {
        return 1.f * (this->t_end - t) / (this->t_end);
    }


    Tyre::Tyre(std::string name) : Function(name) {};


    void Tyre::Configure(const SimFramework::Signal<float> *inRotationalSpeed,
                         const SimFramework::Signal<float> *inLinearSpeed)
     {
        this->m_RotationalSpeed = inRotationalSpeed;
        this->m_LinearSpeed = inLinearSpeed;

        this->SetParameters();
     }

     void Tyre::SetParameters(float radius, float Fz, float B, float C, float D, float E)
     {
        // TODO: Fz could be an input signal
        this->radius = radius;
        this->Fz = Fz;
        this->B = B;
        this->C = C;
        this->D = D;
        this->E = E;
     }

    const SimFramework::Signal<float>*  Tyre::OutForce() const
    {
        return &(this->m_Force);
    };

    const SimFramework::Signal<float>*  Tyre::OutTorque() const
    {
        return &(this->m_Torque);
    };

    std::vector<const SimFramework::SignalBase*> Tyre::InputSignals() const
    {
        return {&(this->m_RotationalSpeed), &(this->m_LinearSpeed)};
    };

    std::vector<const SimFramework::SignalBase*> Tyre::OutputSignals() const
    {
        return {&(this->m_Force), &(this->m_Torque)};
    };

    void Tyre::Update()
    {
        // Input signals
        float V = this->m_LinearSpeed->Read();
        float omega = this->m_RotationalSpeed->Read();

        // Calculate slip ratio
        float V_abs = std::abs(V);
        float V_sx = omega * this->radius - V;
        float k;

        if (std::abs(V) <= this->V_threshold)
        {
            k = (2.f * V_sx) / (V_threshold + (V * V) / V_threshold);
        }
        else
        {
            k = (V_sx) / V_abs;
        }

        float Fx = this->Fz * this->D * std::sin(this->C * std::atan(this->B * k - this->E * (this->B * k - std::atan(this->B * k))));

        // Write result to output signals
        this->m_Force.Write(Fx);
        this->m_Torque.Write(Fx * this->radius);
    };


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
        this->m_Force.Write(0.5 * this->rho * this->A * this->Cd * speed * speed);
    };



    CoulombFriction::CoulombFriction(std::string name) : SimFramework::Function(name) {};

    void CoulombFriction::SetParameters(float mu)
    {
        this->mu = mu;
    };

    void CoulombFriction::Configure(const SimFramework::Signal<float>* inVelocity, const SimFramework::Signal<float>* inNormalForce)
    {
        this->m_Velocity = inVelocity;
        this->m_NormalForce = inNormalForce;
    };

    const SimFramework::Signal<float>* CoulombFriction::OutForce() const
    {
        return &(this->m_Force);
    };

    std::vector<const SimFramework::SignalBase*> CoulombFriction::InputSignals() const
    {
        return {this->m_Velocity, this->m_NormalForce};
    };

    std::vector<const SimFramework::SignalBase*> CoulombFriction::OutputSignals() const
    {
        return {&(this->m_Force)};
    };

    void CoulombFriction::Update()
    {
        // Read inputs
        float speed = this->m_Velocity->Read();
        float normalForce = this->m_NormalForce->Read();

        // Calculate output force
        float output = this->mu * normalForce;

        // Write output
        if (speed < 0)
        {
            output *= -1.f;
        };

        this->m_Force.Write(output);
    };



    Gravity::Gravity(std::string name) : Function(name) {};

    void Gravity::SetParameters(float mass)
    {
        this->mass = mass;
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


    Engine::Engine() : m_EngineMap("Engine Map"), m_TorqueVector("Engine Torque"), m_Inertia("Engine"), m_SpeedMask("Engine") {};

    void Engine::SetParameters(std::string engineJSON, float initialSpeed, float J, float b)
    {
        // Set engine table
        SimFramework::Table3D engineTable = SimFramework::ReadTableJSON(engineJSON, "speed", "throttle", "torque");
        this->m_EngineMap.SetTable(engineTable);

        // Set state space matrices
        Eigen::Matrix<float, 1, 1> A;
        A << -b / J;

        Eigen::Matrix<float, 1, 2> B;
        B << 1.f / J, -1.f / J;

        Eigen::Matrix<float, 1, 1> C;
        C << 1.f;

        Eigen::Matrix<float, 1, 2> D;
        D << 0.f, 0.f;

        this->m_Inertia.SetMatrices(A, B, C, D);

        // Initial engine speed
        Eigen::Matrix<float, 1, 1> init;
        init << initialSpeed;
        this->m_Inertia.SetInitialConditions(init);
    }

    void Engine::Configure(
            const SimFramework::Signal<float>* inThrottle,
            const SimFramework::Signal<float>* inLoadTorque)
    {
        // Configure blocks
        this->m_EngineMap.Configure(this->OutEngineSpeed(), inThrottle);
        this->m_TorqueVector.Configure({this->m_EngineMap.OutSignal(), inLoadTorque});
        this->m_Inertia.Configure(this->m_TorqueVector.OutSignal());
        this->m_SpeedMask.Configure(this->m_Inertia.OutSignal());
    };

    const SimFramework::Signal<float>* Engine::OutEngineSpeed() const
    {
        return this->m_SpeedMask.OutSignal(0);
    };


    SimFramework::BlockList Engine::Blocks()
    {
        // Construct system
        return {{},
                {&(this->m_Inertia)},
                {&(this->m_SpeedMask), &(this->m_EngineMap), &(this->m_TorqueVector)},
                {},
                {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Engine::LogSignals()
    {
        return {{"Engine Speed", this->m_Inertia.OutSignal()},
                {"Engine Torque, Engine Clutch Torque", this->m_TorqueVector.OutSignal()}};
    };


    VehicleDynamics::VehicleDynamics() : m_AeroDrag("Drag"), m_Vectorise("Vehicle Force Input"), m_StateSpace("Vehicle Dynamics"), m_Mask("Vehicle Dynamics") {};

    void VehicleDynamics::SetParameters(float initialPosition, float initialVelocity, float mass, float Cd, float A, float rho) {

        this->m_AeroDrag.SetParameters(Cd, A, rho);
        this->m_Gravity.SetParameters(mass);

        // Set up state matrices
        Eigen::Matrix<float, 2, 2> matA;
        matA << 0.f, 1.f, 0.f, 0.f;

        Eigen::Matrix<float, 2, 3> B;
        B << 0, 0, 0, 1.f / mass, -1.f / mass, -1.f / mass;

        Eigen::Matrix<float, 2, 2> C;
        C << 1.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<float, 2, 3> D;
        D << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;

        this->m_StateSpace.SetMatrices(matA, B, C, D);
        Eigen::Vector2f initialConditions = {initialPosition, initialVelocity};
        this->m_StateSpace.SetInitialConditions(initialConditions);
    };


    void VehicleDynamics::Configure(const SimFramework::Signal<float>* inTyreForce, const SimFramework::Signal<float>* inGradient)
    {
        // Configure blocks
        this->m_AeroDrag.Configure(this->OutVehicleVelocity());
        this->m_Gravity.Configure(inGradient);
        this->m_Vectorise.Configure({inTyreForce, this->m_AeroDrag.OutForce(), this->m_Gravity.OutForce()});
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
        return {
            {},
            {&(this->m_StateSpace)},
            {&(this->m_AeroDrag), &(this->m_Gravity), &(this->m_Vectorise), &(this->m_Mask)},
            {},
            {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > VehicleDynamics::LogSignals()
    {
        return {{"Vehicle Position, Vehicle Velocity", this->m_StateSpace.OutSignal()},
                {"Vehicle Tyre Force, Vehicle Aero Drag, Vehicle Gravity Force", this->m_Vectorise.OutSignal()}};
    };
}; // namespace Models