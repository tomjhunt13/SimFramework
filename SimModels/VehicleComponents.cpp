#include "SimModels/VehicleComponents.h"

namespace Models {

    void Clutch::Configure(SimFramework::Signal<float>* inEngineSpeed, SimFramework::Signal<float>* outClutchTorque)
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

    void Engine::RegisterBlocks(SimFramework::Model* model)
    {
        // Construct system
        model->RegisterBlocks(
                {},
                {&(this->m_BInertia)},
                {&(this->m_BMask), &(this->m_BEngineMap), &(this->m_BSum)},
                {},
                {});
    };



    Transmission::Transmission() {}

    void Transmission::Configure(
            SimFramework::Signal<float>* inClutchTorque,
            SimFramework::Signal<float>* inTyreTorque,
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
        this->m_BBlend.Configure(inClutchTorque, &(this->m_SConst), &(this->m_STrig), &(this->m_SAugmented));
        this->m_BTrig.Configure(&(this->m_STrig));
        this->m_BConst.Configure(&(this->m_SConst), 0);
        this->m_BVec.Configure({&(this->m_SAugmented),  inTyreTorque}, &(this->m_STorqueVec));
        this->m_BStates.Configure(&(this->m_STorqueVec), &(this->m_SSpeeds));
        this->m_BStates.SetInitialConditions(initState);
        this->m_BMask.Configure(&(this->m_SSpeeds), {outClutchSpeed, outTyreSpeed}, {0, 1});
    };

    void Transmission::RegisterBlocks(SimFramework::Model* model)
    {
        model->RegisterBlocks(
                {&(this->m_BConst), &(this->m_BTrig)},
                {&(this->m_BStates)},
                {&(this->m_BBlend), &(this->m_BVec), &(this->m_BMask)},
                {},
                {});
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

        Eigen::Matrix<float, 1,2> B;
        B << 1.f / this->m_EffectiveInertia, - this->m_Ratios[this->m_GearIndex] / this->m_EffectiveInertia;

        Eigen::Matrix<float, 2, 1> C;
        C << 1.f, this->m_Ratios[this->m_GearIndex];

        Eigen::Matrix<float, 2, 2> D;
        D << 0.f, 0.f, 0.f, 0.f;

        this->m_BStates.SetMatrices(A, B, C, D);
    }

    int Transmission::CurrentGear()
    {
        return this->m_GearIndex + 1;
    }

}; // namespace Models