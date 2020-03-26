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


    std::vector<SimFramework::SignalBase*> Clutch::InputSignals()
    {
        return {}; //this->m_InEngineSpeed, this->m_InTransmissionSpeed, this->m_InClutchStiffness};
    };

    std::vector<SimFramework::SignalBase*> Clutch::OutputSignals()
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


    std::vector<SimFramework::SignalBase*> ClutchLowSpeedEngagement::InputSignals()
    {
        return {}; //this->m_InTransmissionSpeed};
    };

    std::vector<SimFramework::SignalBase*> ClutchLowSpeedEngagement::OutputSignals()
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

    std::vector<SimFramework::SignalBase*> Tyre::InputSignals()
    {
        return {};//&(this->m_RotationalSpeed), &(this->m_LinearSpeed)};
    };

    std::vector<SimFramework::SignalBase*> Tyre::OutputSignals()
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


    std::vector<SimFramework::SignalBase*> AeroDrag::InputSignals()
    {
        return {};//this->m_Speed};
    };

    std::vector<SimFramework::SignalBase*> AeroDrag::OutputSignals()
    {
        return {&(this->m_Force)};
    };

    void AeroDrag::Update()
    {
        float speed = this->m_Speed->Read();
        this->m_Force.Write(0.5 * this->rho * this->A * this->Cd * speed * speed);
    };


    void DiscBrake::Configure(SimFramework::Signal<float>* inBrakePressure, SimFramework::Signal<float>* inWheelSpeed, SimFramework::Signal<float>* outBrakeTorque)
    {
        this->m_BrakePressure = inBrakePressure;
        this->m_WheelSpeed = inWheelSpeed;
        this->m_BrakeTorque = outBrakeTorque;

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

    std::vector<SimFramework::SignalBase*> DiscBrake::InputSignals()
    {
        return {this->m_BrakePressure};
    };

    std::vector<SimFramework::SignalBase*> DiscBrake::OutputSignals()
    {
        return {this->m_BrakeTorque};
    };

    void DiscBrake::Update()
    {
        float speed = this->m_WheelSpeed->Read();
        float brakeMagnitude = this->m_BrakeConstant * this->m_BrakePressure->Read();

        if (speed >= 0.f)
        {
            this->m_BrakeTorque->Write(-1 * brakeMagnitude);
        }
        else
        {
            this->m_BrakeTorque->Write(brakeMagnitude);
        };
    };



    void VehicleController::SetParameters(float clutchLagTime, float clutchStiffness)
    {
        this->m_ClutchStiffness = clutchStiffness;
        this->m_BGearChangeTrigger.SetParameters(0.f, clutchLagTime);
    }

    void VehicleController::Configure(
            SimFramework::Signal<float>* inDemandThrottle, SimFramework::Signal<float>* inTransmissionSpeed,
            SimFramework::Signal<float>* outThrottleAugmented, SimFramework::Signal<float>* outClutchStiffness)
    {
        // Configure Blocks
        this->m_BBlendClutchLowSpeed.Configure(&(this->m_SConstZero), &(this->m_SConstClutchStiffness), &(this->m_SEngagementSignal));
        this->m_BBlendThrottle.Configure(inDemandThrottle, &(this->m_SConstZero), &(this->m_STriggerSignal));
        this->m_BBlendClutchGearShift.Configure(&(this->m_SLowSpeedEngangement), &(this->m_SConstZero), &(this->m_STriggerSignal));
        this->m_BLowSpeedEngagement.Configure(inTransmissionSpeed, inDemandThrottle);
        this->m_BConstZero.Configure(0.f);


//        this->m_BClutchStiffnessMax.Configure(&(this->m_SConstClutchStiffness), this->m_ClutchStiffness);
    };

    void VehicleController::Trigger()
    {
        this->m_BGearChangeTrigger.Trigger();
    };

    SimFramework::BlockList VehicleController::Blocks()
    {
        return {{&(this->m_BGearChangeTrigger), &(this->m_BConstZero), &(this->m_BClutchStiffnessMax)},
                {},
                {&(this->m_BBlendClutchLowSpeed), &(this->m_BBlendThrottle), &(this->m_BBlendClutchGearShift), &(m_BLowSpeedEngagement)},
                {},
                {}};
    };


    Engine::Engine() :
            m_SEngineSpeed_("Engine Speed SS Output"), m_SEngineTorque("Engine Map Torque"), m_STorqueInput("Engine Torque Vec"),
            m_BEngineMap("Engine Map"), m_BTorqueVector("Engine Torque Vec"), m_BInertia("Engine"), m_BMask("Engine Mask") {};

    void Engine::SetParameters(std::string engineJSON, float initialSpeed, float J, float b)
    {
        // Set engine table
        SimFramework::Table3D engineTable = SimFramework::ReadTableJSON(engineJSON, "speed", "throttle", "torque");
        this->m_BEngineMap.SetTable(engineTable);

        // Set state space matrices
        Eigen::Matrix<float, 1, 1> A;
        A << -b / J;

        Eigen::Matrix<float, 1, 2> B;
        B << 1.f / J, -1.f / J;

        Eigen::Matrix<float, 1, 1> C;
        C << 1.f;

        Eigen::Matrix<float, 1, 2> D;
        D << 0.f, 0.f;

        this->m_BInertia.SetMatrices(A, B, C, D);

        // Initial engine speed
        Eigen::Matrix<float, 1, 1> init;
        init << initialSpeed;
        this->m_BInertia.SetInitialConditions(init);
    }

    void Engine::Configure(
            SimFramework::Signal<float>* inThrottle,
            SimFramework::Signal<float>* inLoadTorque,
            SimFramework::Signal<float>* outEngineSpeed)
    {
        // Configure blocks
        this->m_BEngineMap.Configure(outEngineSpeed, inThrottle);
        this->m_BTorqueVector.Configure({&(this->m_SEngineTorque), inLoadTorque});
        this->m_BInertia.Configure(&(this->m_STorqueInput));
        this->m_BMask.Configure(&(this->m_SEngineSpeed_));
    };

    SimFramework::BlockList Engine::Blocks()
    {
        // Construct system
        return {{},
                {&(this->m_BInertia)},
                {&(this->m_BMask), &(this->m_BEngineMap), &(this->m_BTorqueVector)},
                {},
                {}};
    };


    void Transmission::SetParameters(
            std::vector<float> gearRatios, float effectiveInertia,
            float Mu, float R, float D, float maxBrakePressure, int N)
    {
        this->m_Ratios = gearRatios;
        this->m_EffectiveInertia = effectiveInertia;
        this->m_BDiscBrake.SetParameters(Mu, R, D, maxBrakePressure, N);
        this->m_GearIndex = 0;
        this->SetGearRatio();

    }

    void Transmission::Configure(
            SimFramework::Signal<float>* inClutchTorque,
            SimFramework::Signal<float>* inTyreTorque,
            SimFramework::Signal<float>* inBrakePressure,
            SimFramework::Signal<float>* outClutchSpeed,
            SimFramework::Signal<float>* outTyreSpeed)
    {

        // TODO: decide where to put this
        //  Initial speed and gear ratio
        Eigen::Vector<float, 1> initState;
        initState << 0;

        // Configure blocks
        this->m_BDiscBrake.Configure(inBrakePressure, outTyreSpeed, &(this->m_SBrakeTorque));
        this->m_BVec.Configure({inClutchTorque,  inTyreTorque, &(this->m_SBrakeTorque)});
        this->m_BStates.Configure(&(this->m_STorqueVec));
        this->m_BStates.SetInitialConditions(initState);
        this->m_BMask.Configure(&(this->m_SSpeeds));
    };

    SimFramework::BlockList Transmission::Blocks()
    {
        return {{},
                {&(this->m_BStates)},
                {&(this->m_BVec), &(this->m_BMask), &(this->m_BDiscBrake)},
                {},
                {}};
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

        std::cout << "A: " << A << ", B: " << B << ", C: " << C << ", D: " << D << std::endl;

        this->m_BStates.SetMatrices(A, B, C, D);
    }

    int Transmission::CurrentGear() const
    {
        return this->m_GearIndex + 1;
    }


    VehicleDynamics::VehicleDynamics() :
        m_SAeroDrag("Drag"), m_SGravity("Const Gravity"), m_SForceVec("Vehicle Force Input"), m_SStatesVec("Vehicle States"),
        m_BAeroDrag("Drag"), m_BVectorise("Vehicle Force Input"), m_BStateSpace("Vehicle Dynamics"), m_BMask("Vehicle Dynamics")
    {};

    void VehicleDynamics::SetParameters(float initialPosition, float initialVelocity, float mass, float Cd, float A, float rho) {
        // Set up state matrices
        Eigen::Matrix<float, 2, 2> matA;
        matA << 0.f, 1.f, 0.f, 0.f;

        Eigen::Matrix<float, 2, 3> B;
        B << 0, 0, 0, 1.f / mass, -1.f / mass, 1.f / mass;

        Eigen::Matrix<float, 2, 2> C;
        C << 1.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<float, 2, 3> D;
        D << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;

        this->m_BStateSpace.SetMatrices(matA, B, C, D);
        Eigen::Vector2f initialConditions = {initialPosition, initialVelocity};
        this->m_BStateSpace.SetInitialConditions(initialConditions);


        // TODO: gravity properly
        this->m_SGravity.Write(0);
    };




    void VehicleDynamics::Configure(
            SimFramework::Signal<float>* inTyreForce,
            SimFramework::Signal<float>* outVehiclePosition,
            SimFramework::Signal<float>* outVehicleVelocity)
    {
        // Configure blocks
        this->m_BAeroDrag.Configure(outVehicleVelocity);
        this->m_BVectorise.Configure({inTyreForce, &(this->m_SAeroDrag), &(this->m_SGravity)});
        this->m_BStateSpace.Configure(&(this->m_SForceVec));
        this->m_BMask.Configure(&(this->m_SStatesVec));


    };

    SimFramework::BlockList VehicleDynamics::Blocks()
    {
        return {
            {},
            {&(this->m_BStateSpace)},
            {&(this->m_BAeroDrag), &(this->m_BVectorise), &(this->m_BMask)},
            {},
            {}};
    };
}; // namespace Models