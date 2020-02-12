#include "Block.h"

namespace SimFramework {

    Block::Block() {
        SystemManager::RegisterBlock(this);
    }

} // namespace SimFramework