// Optional Bullet translation unit.
// When WQS_USE_BULLET is defined, collision_world.cpp already contains the
// Bullet path. This file exists so CMake can list a dedicated backend unit
// and so a custom Bullet-only build can swap implementations later.
//
// Default builds (no WQS_USE_BULLET) compile a no-op here.

#include "collision_world/collision_world.h"

#ifndef WQS_USE_BULLET
namespace wqs {
void bulletBackendLinked() {}
}
#endif
