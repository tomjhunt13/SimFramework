#ifndef SIMINTERFACE_BLOCK_H
#define SIMINTERFACE_BLOCK_H

#include <vector>
#include "Signal.h"

class BlockManager;

class Block {
public:

    Block(BlockManager& manager);


    virtual void Read() = 0;
    virtual void Write() = 0;

    virtual void Update(float finalTime) = 0;
};


class BlockManager {

private:
    std::vector<Block*> m_Blocks;

public:
    void RegisterBlock(Block* block);

    void UpdateSystem(float tMax);

};

#endif //SIMINTERFACE_BLOCK_H
