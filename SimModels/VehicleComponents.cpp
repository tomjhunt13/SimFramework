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


    ClutchLowSpeedEngagement::ClutchLowSpeedEngagement(float threshold, std::string name) :
        Function(name), m_SpeedThreshold(threshold), m_Accelerating(false) {};

    void ClutchLowSpeedEngagement::Configure(
            const SimFramework::Signal<float>* inTransmissionSpeed,
            const SimFramework::Signal<float>* inThrottle)
    {
        this->m_InTransmissionSpeed = inTransmissionSpeed;
        this->m_InThrottle = inThrottle;
    };

    const SimFramework::Signal<float>* ClutchLowSpeedEngagement::OutEngagement() const
    {
        return &(this->m_OutEngagement);
    };


    std::vector<const SimFramework::SignalBase*> ClutchLowSpeedEngagement::InputSignals() const
    {
        return {this->m_InTransmissionSpeed};
    };

    std::vector<const SimFramework::SignalBase*> ClutchLowSpeedEngagement::OutputSignals() const
    {
        return {&(this->m_OutEngagement)};
    };

    void ClutchLowSpeedEngagement::Update()
    {
        float throttle = this->m_InThrottle->Read();
        float speed = this->m_InTransmissionSpeed->Read();

        this->m_Accelerating = (throttle < this->m_ThrottleThreshold) ? false : true;

        float engagement = 1;


        // TODO: Tidy up!

        if (speed > this->m_SpeedThreshold)
        {
            engagement = 1;
        };

        if (speed <= 0)
        {
            engagement = 0;
        }


        if ((speed < this->m_SpeedThreshold) && this->m_Accelerating)
        {
            engagement = std::abs(speed / this->m_SpeedThreshold) * 0.5 + 0.5;
        };

        if ((speed < this->m_SpeedThreshold) && !(this->m_Accelerating))
        {
            engagement = 0;
        };


        this->m_OutEngagement.Write(engagement);
    };


//    CentrifugalClutch::CentrifugalClutch(std::string name) : Function(name) {};
//
//    void CentrifugalClutch::Configure(SimFramework::Signal<float>* inEngineSpeed, SimFramework::Signal<float>* outClutchTorque)
//    {
//        this->m_InEngineSpeed = inEngineSpeed;
//        this->m_OutClutchTorque = outClutchTorque;
//    };
//
//    std::vector<SimFramework::SignalBase*> CentrifugalClutch::InputSignals()
//    {
//        return {this->m_InEngineSpeed};
//    }
//
//    std::vector<SimFramework::SignalBase*> CentrifugalClutch::OutputSignals()
//    {
//        return {this->m_OutClutchTorque};
//    }
//
//    void CentrifugalClutch::Update()
//    {
//        float speed = SimFramework::RadiansPerSecondToRPM(this->m_InEngineSpeed->Read()) / 1000.f;
//        this->m_OutClutchTorque->Write(this->m_TorqueCapacity * ( speed * speed - this->m_EngagementSpeed * this->m_EngagementSpeed));
//    };

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


    void DiscBrake::Configure(const SimFramework::Signal<float>* inBrakePressure, const SimFramework::Signal<float>* inWheelSpeed)
    {
        this->m_BrakePressure = inBrakePressure;
        this->m_WheelSpeed = inWheelSpeed;

        this->SetParameters();
    };

    void DiscBrake::SetParameters(float mu, float R, float D, float maxBrakePressure, int N)
    {
        this->mu = mu;
        this->R = R;
        this->D = D;
        this->maxBrakePressure = maxBrakePressure;
        this->N = N;

        this->m_BrakeConstant = (mu * SimFramework::pi() * D * D * R * N * maxBrakePressure); // For all four wheels
    };

    const SimFramework::Signal<float>* DiscBrake::OutTorque() const
    {
        return &(this->m_BrakeTorque);
    };


    std::vector<const SimFramework::SignalBase*> DiscBrake::InputSignals() const
    {
        return {this->m_BrakePressure};
    };

    std::vector<const SimFramework::SignalBase*> DiscBrake::OutputSignals() const
    {
        return {&(this->m_BrakeTorque)};
    };

    void DiscBrake::Update()
    {
        float speed = this->m_WheelSpeed->Read();
        float brakeMagnitude = this->m_BrakeConstant * this->m_BrakePressure->Read();

        if (speed >= 0.f)
        {
            this->m_BrakeTorque.Write(-1 * brakeMagnitude);
        }
        else
        {
            this->m_BrakeTorque.Write(brakeMagnitude);
        };
    };



    void VehicleController::SetParameters(float clutchLagTime, float clutchStiffness)
    {
        this->m_ClutchStiffness = clutchStiffness;
        this->m_GearChangeTrigger.SetParameters(0.f, clutchLagTime);
    }

    void VehicleController::Configure(
            const SimFramework::Signal<float>* inDemandThrottle,
            const SimFramework::Signal<float>* inTransmissionSpeed)
    {
        // Configure Blocks
        this->m_BlendClutchLowSpeed.Configure(this->m_ConstZero.OutSignal(), this->m_ClutchStiffnessMax.OutSignal(), this->m_LowSpeedEngagement.OutEngagement());
        this->m_BlendThrottle.Configure(inDemandThrottle, this->m_ConstZero.OutSignal(), this->m_GearChangeTrigger.OutSignal());
        this->m_BlendClutchGearShift.Configure(this->m_LowSpeedEngagement.OutEngagement(), this->m_ConstZero.OutSignal(), this->m_GearChangeTrigger.OutSignal());
        this->m_LowSpeedEngagement.Configure(inTransmissionSpeed, inDemandThrottle);
        this->m_ConstZero.Configure(0.f);
        this->m_ClutchStiffnessMax.Configure(this->m_ClutchStiffness);
    };


    const SimFramework::Signal<float>* VehicleController::OutAugmentedThrottle() const
    {
        return this->m_BlendThrottle.OutSignal();
    };

    const SimFramework::Signal<float>* VehicleController::OutClutchStiffness() const
    {
        return this->m_BlendClutchGearShift.OutSignal();
    };

    void VehicleController::Trigger()
    {
        this->m_GearChangeTrigger.Trigger();
    };

    SimFramework::BlockList VehicleController::Blocks()
    {
        return {{&(this->m_GearChangeTrigger), &(this->m_ConstZero), &(this->m_ClutchStiffnessMax)},
                {},
                {&(this->m_BlendClutchLowSpeed), &(this->m_BlendThrottle), &(this->m_BlendClutchGearShift), &(m_LowSpeedEngagement)},
                {},
                {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > VehicleController::LogSignals()
    {
        return {{"Clutch Stiffness", this->OutClutchStiffness()},
                {"Augmented Throttle", this->OutAugmentedThrottle()}};
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


    void Transmission::SetParameters(
            std::vector<float> gearRatios, float effectiveInertia,
            float Mu, float R, float D, float maxBrakePressure, int N)
    {
        this->m_Ratios = gearRatios;
        this->m_EffectiveInertia = effectiveInertia;
        this->m_DiscBrake.SetParameters(Mu, R, D, maxBrakePressure, N);

        // Initialise dynamic variables and initial conditions
        this->m_GearIndex = 0;
        this->SetGearRatio();

        Eigen::Vector<float, 1> initialConditions;
        initialConditions << 0.f;
        this->m_States.SetInitialConditions(initialConditions);
    }

    void Transmission::Configure(
            const SimFramework::Signal<float>* inClutchTorque,
            const SimFramework::Signal<float>* inTyreTorque,
            const SimFramework::Signal<float>* inBrakePressure)
    {
        // Configure blocks
        this->m_DiscBrake.Configure(inBrakePressure, this->OutTyreSpeed());
        this->m_TorqueVector.Configure({inClutchTorque, inTyreTorque, this->m_DiscBrake.OutTorque()});
        this->m_States.Configure(this->m_TorqueVector.OutSignal());
        this->m_StateMask.Configure(this->m_States.OutSignal());
    };

    const SimFramework::Signal<float>* Transmission::OutClutchSpeed() const
    {
        return this->m_StateMask.OutSignal(0);
    };

    const SimFramework::Signal<float>* Transmission::OutTyreSpeed() const
    {
        return this->m_StateMask.OutSignal(1);
    };


    SimFramework::BlockList Transmission::Blocks()
    {
        return {{},
                {&(this->m_States)},
                {&(this->m_TorqueVector), &(this->m_StateMask), &(this->m_DiscBrake)},
                {},
                {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > Transmission::LogSignals()
    {
        return {{"Transmission Clutch Speed, Transmission Wheel Speed", this->m_States.OutSignal()},
                {"Transmission Clutch Torque, Transmission Tyre Torque, Transmission Brake Torque", this->m_TorqueVector.OutSignal()}};
    };

    bool Transmission::ShiftUp()
    {
        // Ignore if in top gear
        if (this->m_GearIndex == this->m_Ratios.size() - 1)
        {
            return false;
        }

        // Else increment gear
        this->m_GearIndex += 1;

        // Change gear
        this->SetGearRatio();

        return true;
    };

    bool Transmission::ShiftDown()
    {
        // Ignore if in bottom gear
        if (this->m_GearIndex == 0)
        {
            return false;
        }

        // Else decrement gear
        this->m_GearIndex -= 1;

        // Change gear
        this->SetGearRatio();

        return true;
    };

    void Transmission::SetGearRatio()
    {
        Eigen::Matrix<float, 1, 1> A;
        A << 0.f;

        Eigen::Matrix<float, 1, 3> B;
        B << 1.f / this->m_EffectiveInertia, - this->m_Ratios[this->m_GearIndex] / this->m_EffectiveInertia, this->m_Ratios[this->m_GearIndex] / this->m_EffectiveInertia;

        Eigen::Matrix<float, 2, 1> C;
        C << 1.f, this->m_Ratios[this->m_GearIndex];

        Eigen::Matrix<float, 2, 3> D;
        D << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;

        this->m_States.SetMatrices(A, B, C, D);
    }

    int Transmission::CurrentGear() const
    {
        return this->m_GearIndex + 1;
    }


    VehicleDynamics::VehicleDynamics() : m_AeroDrag("Drag"), m_Vectorise("Vehicle Force Input"), m_StateSpace("Vehicle Dynamics"), m_Mask("Vehicle Dynamics") {};

    void VehicleDynamics::SetParameters(float initialPosition, float initialVelocity, float mass, float Cd, float A, float rho) {

        this->m_AeroDrag.SetParameters(Cd, A, rho);

        // Set up state matrices
        Eigen::Matrix<float, 2, 2> matA;
        matA << 0.f, 1.f, 0.f, 0.f;

        Eigen::Matrix<float, 2, 2> B;
        B << 0, 0, 1.f / mass, -1.f / mass;

        Eigen::Matrix<float, 2, 2> C;
        C << 1.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<float, 2, 2> D;
        D << 0.f, 0.f, 0.f, 0.f;

        this->m_StateSpace.SetMatrices(matA, B, C, D);
        Eigen::Vector2f initialConditions = {initialPosition, initialVelocity};
        this->m_StateSpace.SetInitialConditions(initialConditions);
    };


    void VehicleDynamics::Configure(const SimFramework::Signal<float>* inTyreForce)
    {
        // Configure blocks
        this->m_AeroDrag.Configure(this->OutVehicleVelocity());
        this->m_Vectorise.Configure({inTyreForce, this->m_AeroDrag.OutForce()});
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
            {&(this->m_AeroDrag), &(this->m_Vectorise), &(this->m_Mask)},
            {},
            {}};
    };

    std::vector<std::pair<std::string, const SimFramework::SignalBase *> > VehicleDynamics::LogSignals()
    {
        return {{"Vehicle Position, Vehicle Velocity", this->m_StateSpace.OutSignal()},
                {"Vehicle Tyre Force, Vehicle Aero Drag", this->m_Vectorise.OutSignal()}};
    };
}; // namespace Models