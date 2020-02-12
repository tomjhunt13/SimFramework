#ifndef SIMINTERFACE_SYSTEMMANAGER_H
#define SIMINTERFACE_SYSTEMMANAGER_H


#include <vector>
#include "Block.h"

namespace SimFramework {

    class Block;

    class SystemManager {

    public:

        // Copy constructor deleted according to singleton design pattern
        SystemManager(SystemManager &systemManager) = delete;

        static SystemManager& Get();
        static void RegisterBlock(Block* block);
        static void UpdateSystem(float tMax);


    private:

        // Reference to blocks in system
        std::vector<Block*> m_Blocks;

        // Constructor hidden to maintain singleton pattern
        SystemManager() {};
    };


} // namespace SimFramework

#endif //SIMINTERFACE_SYSTEMMANAGER_H
