#include "SimModels/VehicleComponents.h"

namespace Models {

    void Clutch::Configure(SimFramework::Signal<float>* inEngineSpeed, SimFramework::Signal<float>* inClutchSpeed, SimFramework::Signal<float>* outClutchTorque)
    {
        this->m_InEngineSpeed = inEngineSpeed;
        this->m_OutClutchTorque = outClutchTorque;
    };

    std::vector<SimFramework::SignalBase*> Clutch::InputSignals()
    {
        return {this->m_InEngineSpeed};
    }

    std::vector<SimFramework::SignalBase*> Clutch::OutputSignals()
    {
        return {this->m_OutClutchTorque};
    }

    void Clutch::Update()
    {
        float speed = SimFramework::RadiansPerSecondToRPM(this->m_InEngineSpeed->Read()) / 1000.f;
        this->m_OutClutchTorque->Write(this->m_TorqueCapacity * ( speed * speed - this->m_EngagementSpeed * this->m_EngagementSpeed));
    };


    LinearTrigger::LinearTrigger()
    {
        this->m_Default = 0.f;
        this->t_end = 1.f;
    }

    float LinearTrigger::Evaluate(float t) {
        return 1.f * (this->t_end - t) / (this->t_end);
    }


    void Tyre::Configure(SimFramework::Signal<float> *inRotationalSpeed,
                         SimFramework::Signal<float> *inLinearSpeed,
                         SimFramework::Signal<float> *outForce,
                         SimFramework::Signal<float> *outTorque)
     {
        this->m_RotationalSpeed = inRotationalSpeed;
        this->m_LinearSpeed = inLinearSpeed;
        this->m_Force = outForce;
        this->m_Torque = outTorque;

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

    std::vector<SimFramework::SignalBase*> Tyre::InputSignals()
    {
        return {&(this->m_RotationalSpeed), &(this->m_LinearSpeed)};
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
        float k = (omega * this->radius - V) / std::abs(V);
        float Fx = this->Fz * this->D * std::sin(this->C * std::atan(this->B * k - this->E * (this->B * k - std::atan(this->B * k))));

        // Write result to output signals
        this->m_Force->Write(Fx);
        this->m_Torque->Write(Fx * this->radius);
    };


    void AeroDrag::Configure(SimFramework::Signal<float>* inSpeed, SimFramework::Signal<float>* outForce)
    {
        this->m_Speed = inSpeed;
        this->m_Force = outForce;
        this->SetParameters();
    };

    void AeroDrag::SetParameters(float Cd, float A, float rho)
    {
        this->Cd = Cd;
        this->A = A;
        this->rho = rho;
    };

    std::vector<SimFramework::SignalBase*> AeroDrag::InputSignals()
    {
        return {this->m_Speed};
    };

    std::vector<SimFramework::SignalBase*> AeroDrag::OutputSignals()
    {
        return {this->m_Force};
    };

    void AeroDrag::Update()
    {
        float speed = this->m_Speed->Read();
        this->m_Force->Write(0.5 * this->rho * this->A * this->Cd * speed * speed);
    };


    void DiscBrake::Configure(SimFramework::Signal<float>* inBrakePressure, SimFramework::Signal<float>* inWheelSpeed, SimFramework::Signal<float>* outBrakeTorque)
    {
        this->m_BrakePressure = inBrakePressure;
        this->m_WheelSpeed = inWheelSpeed;
        this->m_BrakeTorque = outBrakeTorque;

        this->SetParameters();
    };

    void DiscBrake::SetParameters(float mu, float R, float D, int N)
    {
        this->mu = mu;
        this->R = R;
        this->D = D;
        this->N = N;

        this->m_BrakeConstant = (mu * SimFramework::pi() * D * D * R * N) / (4);
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
        float brakeMagnitude = this->m_BrakeConstant * this->m_BrakePressure->Read() * 6000000.f;

        if (speed >= 0.f)
        {
            this->m_BrakeTorque->Write(-1 * brakeMagnitude);
        }
        else
        {
            this->m_BrakeTorque->Write(brakeMagnitude);
        };
    };


    void Engine::SetEngineParameters(std::string engineJSON, float J, float b)
    {
        // Set engine table
        SimFramework::Table3D engineTable = SimFramework::ReadTableJSON(engineJSON, "speed", "throttle", "torque");
        this->m_BEngineMap.SetTable(engineTable);

        // Set state space matrices
        Eigen::Matrix<float, 1, 1> A;
        A << -b;

        Eigen::Matrix<float, 1, 1> B;
        B << 1.f / J;

        Eigen::Matrix<float, 1, 1> C;
        C << 1.f;

        Eigen::Matrix<float, 1, 1> D;
        D << 1.f;

        this->m_BInertia.SetMatrices(A, B, C, D);

        // Initial engine speed
        Eigen::Matrix<float, 1, 1> init;
        init << 300.f;
        this->m_BInertia.SetInitialConditions(init);
    }

    void Engine::Configure(
            SimFramework::Signal<float>* inThrottle,
            SimFramework::Signal<float>* inLoadTorque,
            SimFramework::Signal<float>* outEngineSpeed)
    {
        // Configure blocks
        this->m_BEngineMap.Configure(outEngineSpeed, inThrottle, &(this->m_SEngineTorque));
        this->m_BSum.Configure({&(this->m_SEngineTorque), inLoadTorque}, &(this->m_SResultantTorque), {1.f, -1.f});
        this->m_BInertia.Configure(&(this->m_SResultantTorque), &(this->m_SEngineSpeed_));
        this->m_BMask.Configure(&(this->m_SEngineSpeed_), {outEngineSpeed}, {0});
    };

    SimFramework::BlockList Engine::Blocks()
    {
        // Construct system
        return {{},
                {&(this->m_BInertia)},
                {&(this->m_BMask), &(this->m_BEngineMap), &(this->m_BSum)},
                {},
                {}};
    };


    void Transmission::Configure(
            SimFramework::Signal<float>* inClutchTorque,
            SimFramework::Signal<float>* inTyreTorque,
            SimFramework::Signal<float>* inBrakePressure,
            SimFramework::Signal<float>* outClutchSpeed,
            SimFramework::Signal<float>* outTyreSpeed)
    {

        // TODO: decide where to put this
        //  Initial speed and gear ratio
        this->m_GearIndex = 0;
        this->SetGearRatio(this->m_GearIndex);
        Eigen::Vector<float, 1> initState;
        initState << 0;

        // Configure blocks
        this->m_BDiscBrake.Configure(inBrakePressure, outTyreSpeed, &(this->m_SBrakeTorque));
        this->m_BBlend.Configure(inClutchTorque, &(this->m_SConst), &(this->m_STrig), &(this->m_SAugmented));
        this->m_BTrig.Configure(&(this->m_STrig));
        this->m_BConst.Configure(&(this->m_SConst), 0);
        this->m_BVec.Configure({&(this->m_SAugmented),  inTyreTorque, &(this->m_SBrakeTorque)}, &(this->m_STorqueVec));
        this->m_BStates.Configure(&(this->m_STorqueVec), &(this->m_SSpeeds));
        this->m_BStates.SetInitialConditions(initState);
        this->m_BMask.Configure(&(this->m_SSpeeds), {outClutchSpeed, outTyreSpeed}, {0, 1});
    };

    SimFramework::BlockList Transmission::Blocks()
    {
        return {{&(this->m_BConst), &(this->m_BTrig)},
                {&(this->m_BStates)},
                {&(this->m_BBlend), &(this->m_BVec), &(this->m_BMask), &(this->m_BDiscBrake)},
                {},
                {}};
    };


    void Transmission::ShiftUp()
    {
        // Ignore if in top gear
        if (this->m_GearIndex == this->m_Ratios.size() - 1)
        {
            return;
        }

        // Else increment gear
        this->m_GearIndex += 1;

        // Trigger trigger  block
        this->m_BTrig.Trigger();

        // Change gear
        this->SetGearRatio(this->m_GearIndex);

    };
    void Transmission::ShiftDown()
    {
        // Ignore if in bottom gear
        if (this->m_GearIndex == 0)
        {
            return;
        }

        // Else decrement gear
        this->m_GearIndex -= 1;

        // Trigger trigger block
        this->m_BTrig.Trigger();

        // Change gear
        this->SetGearRatio(this->m_GearIndex);
    };

    void Transmission::SetGearRatio(int gearIndex)
    {
        Eigen::Matrix<float, 1, 1> A;
        A << 0.f;

        Eigen::Matrix<float, 1,3> B;
        B << 1.f / this->m_EffectiveInertia, - this->m_Ratios[this->m_GearIndex] / this->m_EffectiveInertia, this->m_Ratios[this->m_GearIndex] / this->m_EffectiveInertia;

        Eigen::Matrix<float, 2, 1> C;
        C << 1.f, this->m_Ratios[this->m_GearIndex];

        Eigen::Matrix<float, 2, 3> D;
        D << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;

        this->m_BStates.SetMatrices(A, B, C, D);
    }

    int Transmission::CurrentGear()
    {
        return this->m_GearIndex + 1;
    }


    void VehicleDynamics::Configure(
            SimFramework::Signal<float>* inTyreForce,
            SimFramework::Signal<float>* outVehiclePosition,
            SimFramework::Signal<float>* outVehicleVelocity)
    {
        // Configure blocks
        this->m_BAeroDrag.Configure(outVehicleVelocity, &(this->m_SAeroDrag));
        this->m_BVectorise.Configure({inTyreForce, &(this->m_SAeroDrag), &(this->m_SGravity)}, &(this->m_SForceVec));
        this->m_BStateSpace.Configure(&(this->m_SForceVec), &(this->m_SStatesVec));
        this->m_BMask.Configure(&(this->m_SStatesVec), {outVehiclePosition, outVehicleVelocity}, {0, 1});

        // Set up state matrices
        Eigen::Matrix<float, 2, 2> A;
        A << 0.f, 1.f, 0.f, 0.f;

        Eigen::Matrix<float, 2, 3> B;
        B << 0, 0, 0, 1.f / this->mass, -1.f / this->mass, 1.f / this->mass;

        Eigen::Matrix<float, 2, 2> C;
        C << 1.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<float, 2, 3> D;
        D << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;

        this->m_BStateSpace.SetMatrices(A, B, C, D);
        Eigen::Vector2f initialConditions = {0.f, 0.f};
        this->m_BStateSpace.SetInitialConditions(initialConditions);


        // TODO: gravity properly
        this->m_SGravity.Write(0);
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