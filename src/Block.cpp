#include "Block.h"

Block::Block(BlockManager& manager) {
    manager.RegisterBlock(this);
}

void BlockManager::RegisterBlock(Block* block) {
    this->m_Blocks.push_back(block);
};