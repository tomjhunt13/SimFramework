#ifndef SIMFRAMEWORK_FRAMEWORK_H
#define SIMFRAMEWORK_FRAMEWORK_H

#include <vector>
#include <string>

#include "Eigen/Dense"

namespace SimFramework {

    class Block;

    //----------------- Signal
    template <int length>
    class Signal
    {
    public:
        explicit Signal(Block* inputBlock, std::string name = "Signal") : m_InputBlock(inputBlock), m_Name(name) {};

        Eigen::Vector<float, length> Read() const { return this->m_Value; };
        void Write(Eigen::Vector<float, length> value) { this->m_Value = value; };
        Block* InputBlock() { return m_InputBlock; };

    private:
        Eigen::Vector<float, length> m_Value;
        Block* m_InputBlock;
        std::string m_Name;
    };


    //----------------- Blocks
    enum e_BlockType { Source, DynamicSystem, Function, Sink};

    class Block
    {
    public:
        Block(e_BlockType blockType);

        virtual void Read() = 0;
        virtual void Update(float t_np1) = 0;
        virtual void Write() = 0;

    private:
        unsigned int m_ID;
        e_BlockType m_BlockType;
    };




    //----------------- SystemManager
    class SystemManager {

    public:

        // Copy constructor deleted according to singleton design pattern
        SystemManager(SystemManager &systemManager) = delete;

        static SystemManager& Get();

        // Construction phase
        static unsigned int RegisterBlock(Block* block);

        // Initialisation phase



        // Solution phase
        static void UpdateSystem(float tMax);


    private:

        // Reference to blocks in system
        std::vector<Block*> m_Blocks;

        // Constructor hidden to maintain singleton pattern
        SystemManager() = default;
    };

} // namespace SimFramework

#endif //SIMFRAMEWORK_FRAMEWORK_H
