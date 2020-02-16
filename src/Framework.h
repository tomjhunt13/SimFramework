#ifndef SIMFRAMEWORK_FRAMEWORK_H
#define SIMFRAMEWORK_FRAMEWORK_H

#include <vector>
#include <map>
#include <string>

#include "Eigen/Dense"

namespace SimFramework {

    // Forward Declarations
    class Block;


    //----------------- Signal
    class Signal
    {
    public:
//        Signal() {};
        Signal(std::string name = "Signal") : m_Name(name) {};

        Eigen::VectorXf Read() const { return this->m_Value; };
        void Write(Eigen::VectorXf value) { this->m_Value = value; };

        Block* GetInputBlock() { return m_InputBlock; };
        void SetInputBlock(Block* inputBlock) { this->m_InputBlock = inputBlock; };

    private:
        Eigen::VectorXf m_Value;
        Block* m_InputBlock;
        std::string m_Name;
    };


    //----------------- Blocks
    enum e_BlockType { eSource, eDynamicSystem, eFunction, eSink};

    class Block
    {
    public:
        Block(e_BlockType blockType);

        e_BlockType BlockType() { return this->m_BlockType; };

        // List of blocks driving input signals
        std::vector<Block*> InputBlocks();

        // Block API
        virtual void Update(float t_np1) = 0;

        //
        void Read();
        void Write();

    protected:

        // Signal Registration
        void RegisterInputSignal(Signal* inputSignal);
        void RegisterOutputSignal(Signal* outputSignal);

        std::vector<Eigen::VectorXf> m_InputCopy;
        Eigen::VectorXf m_OutputCopy;

    private:

        e_BlockType m_BlockType;
        std::vector<Signal*> m_InputSignals;
        std::vector<Signal*> m_OutputSignals;
    };

    class Source : public Block {
    public:
        Source() : Block(e_BlockType::eSource) {};

    protected:
        float t_n;
    };

    class DynamicSystem : public Block {
    public:
        DynamicSystem() : Block(e_BlockType::eDynamicSystem) {};

        virtual Eigen::VectorXf Gradient(float t, Eigen::VectorXf x) = 0;

    protected:
        float t_n;
    };

    class Function : public Block {
    public:
        Function() : Block(e_BlockType::eFunction) {};
    };

    class Sink : public Block {
    public:
        Sink() : Block(e_BlockType::eSink) {};

    protected:
        float t_n;
    };




    //----------------- SystemManager


    class SystemManager {

    public:

        // Copy constructor deleted according to singleton design pattern
        SystemManager(SystemManager &systemManager) = delete;

        static SystemManager& Get();

        // Construction phase
        static void RegisterBlock(Block* block);

        // Initialisation phase
        void ConstructSystem();

        // Solution phase
        static void UpdateSystem(float t_np1);

    private:

        // Helper methods
        void OrderFunctions();

        // Reference to blocks in system
        std::vector<Block*> m_Sources;
        std::vector<Block*> m_DynamicSystems;
        std::vector<Block*> m_Functions;
        std::vector<Block*> m_Sinks;

        // Constructor hidden to maintain singleton pattern
        SystemManager() = default;
    };


    class ForwardEuler {
    public:
        static Eigen::VectorXf Step(DynamicSystem & block, float dt, float t, Eigen::VectorXf &x_n);
    };

} // namespace SimFramework

#endif //SIMFRAMEWORK_FRAMEWORK_H
