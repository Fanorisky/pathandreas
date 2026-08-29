cmake_minimum_required(VERSION 3.16)
project(world-query-service LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(WQS_USE_BULLET "Use Bullet Physics (btBvhTriangleMeshShape) instead of the built-in SAH BVH" OFF)
option(WQS_BUILD_TESTS "Build unit tests" ON)

set(WQS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# --- Recast / Detour (vendored) ---
set(RECAST_DIR ${WQS_ROOT}/third_party/recastnavigation)
file(GLOB RECAST_SRC ${RECAST_DIR}/Recast/Source/*.cpp)
file(GLOB DETOUR_SRC ${RECAST_DIR}/Detour/Source/*.cpp)
add_library(recast_detour STATIC ${RECAST_SRC} ${DETOUR_SRC})
target_include_directories(recast_detour PUBLIC
    ${RECAST_DIR}/Recast/Include
    ${RECAST_DIR}/Detour/Include)

# --- Optional Bullet ---
if(WQS_USE_BULLET)
    include(FetchContent)
    FetchContent_Declare(bullet3
        GIT_REPOSITORY https://github.com/bulletphysics/bullet3.git
        GIT_TAG 3.25
        GIT_SHALLOW TRUE)
    set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
    set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
    set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "" FORCE)
    set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)
    set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
    set(BUILD_PYBULLET OFF CACHE BOOL "" FORCE)
    set(USE_GRAPHICAL_BENCHMARK OFF CACHE BOOL "" FORCE)
    set(INSTALL_LIBS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(bullet3)
endif()

set(WQS_SRC
    src/collision_loader/tessellate.cpp
    src/collision_loader/cadb.cpp
    src/collision_loader/col.cpp
    src/collision_loader/test_city.cpp
    src/collision_world/bvh.cpp
    src/collision_world/collision_world.cpp
    src/collision_world/bullet_backend.cpp
    src/navmesh_builder/navmesh_file.cpp
    src/navmesh_builder/navmesh_builder.cpp
    src/pathfinder/pathfinder.cpp
    src/query_server/json_protocol.cpp
    src/query_server/query_server.cpp
)

add_library(wqs STATIC ${WQS_SRC})
target_include_directories(wqs PUBLIC
    ${WQS_ROOT}/src
    ${WQS_ROOT}/third_party)
target_link_libraries(wqs PUBLIC recast_detour pthread)
if(WQS_USE_BULLET)
    target_compile_definitions(wqs PUBLIC WQS_USE_BULLET)
    target_link_libraries(wqs PUBLIC BulletCollision BulletDynamics LinearMath)
    target_include_directories(wqs PUBLIC
        ${bullet3_SOURCE_DIR}/src)
endif()

add_executable(world-query-service src/main.cpp)
target_link_libraries(world-query-service PRIVATE wqs)

add_executable(navmesh_builder tools/navmesh_builder_cli.cpp)
target_link_libraries(navmesh_builder PRIVATE wqs)

if(WQS_BUILD_TESTS)
    add_executable(wqs_tests tests/test_all.cpp)
    target_link_libraries(wqs_tests PRIVATE wqs)
endif()
