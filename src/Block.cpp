#include "Block.h"

Block::Block(TimeManager& manager) {
    manager.RegisterBlock(this);
}