#include "Block.h"

namespace SimInterface {

    Block::Block() {
        SystemManager::RegisterBlock(this);
    }

} // namespace SimInterface