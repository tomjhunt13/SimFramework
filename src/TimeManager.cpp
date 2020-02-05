//
// Created by Thomas Hunt on 05/02/2020.
//

#include "TimeManager.h"

void TimeManager::RegisterBlock(Block* block) {
    this->m_Blocks.push_back(block);
};