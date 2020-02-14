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
    enum e_BlockType { Source, DynamicSystem, Function, Sink};

    class Block
    {
    public:
        Block(e_BlockType blockType);

        e_BlockType BlockType() { return this->m_BlockType; };

        // List of blocks driving input signals
        std::vector<Block*> InputBlocks();

        // Block API
        virtual void Read() = 0;
        virtual void Update(float t_np1) = 0;
        virtual void Write() = 0;

    protected:

        // Signal Registration
        void RegisterInputSignal(Signal& inputSignal);
        void RegisterOutputSignal(Signal& outputSignal);

    private:

        e_BlockType m_BlockType;
        std::vector<Signal*> m_InputSignals;
        std::vector<Signal*> m_OutputSignals;
    };

    class DynamicSystem : public Block {
    public:
        DynamicSystem() : Block(e_BlockType::DynamicSystem) {};
    };

    class Function : public Block {
    public:
        Function() : Block(e_BlockType::Function) {};
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

} // namespace SimFramework

#endif //SIMFRAMEWORK_FRAMEWORK_H
