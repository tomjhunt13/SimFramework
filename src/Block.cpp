#include "Block.h"

Block::Block(BlockManager& manager) {
    manager.RegisterBlock(this);
}

void BlockManager::RegisterBlock(Block* block) {
    this->m_Blocks.push_back(block);
};

void BlockManager::UpdateSystem(float tMax) {

    // Read inputs
    for (auto i: this->m_Blocks)
    {
        i->Read();
    }

    // Update
    for (auto i: this->m_Blocks)
    {
        i->Update(tMax);
    }

    // Write outputs
    for (auto i: this->m_Blocks)
    {
        i->Write();
    }

}