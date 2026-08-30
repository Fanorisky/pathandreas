# Makefile for environments without CMake. Default backend is the built-in BVH.
# For Bullet: cmake -DWQS_USE_BULLET=ON ..

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -pthread
# 64-bit polygon references remove Detour's per-tile poly-count ceiling
# (tileBits + polyBits <= 22 under 32-bit refs). With ~1444 tiles the 32-bit
# budget caps a tile at 2048 polys and addTile silently drops the densest
# city tiles. Must be defined consistently for every TU that includes
# DetourNavMesh.h - it changes the dtPolyRef type.
CXXFLAGS += -DDT_POLYREF64
INCLUDES  = -Isrc -Ithird_party \
            -Ithird_party/recastnavigation/Recast/Include \
            -Ithird_party/recastnavigation/Detour/Include
LDFLAGS   = -pthread

RECAST_SRC := $(wildcard third_party/recastnavigation/Recast/Source/*.cpp)
DETOUR_SRC := $(wildcard third_party/recastnavigation/Detour/Source/*.cpp)

WQS_SRC := \
    src/collision_loader/tessellate.cpp \
    src/collision_loader/cadb.cpp \
    src/collision_loader/col.cpp \
    src/collision_loader/test_city.cpp \
    src/collision_world/bvh.cpp \
    src/collision_world/collision_world.cpp \
    src/collision_world/bullet_backend.cpp \
    src/navmesh_builder/navmesh_file.cpp \
    src/navmesh_builder/navmesh_builder.cpp \
    src/pathfinder/pathfinder.cpp \
    src/road_network/road_network.cpp \
    src/road_network/sa_paths.cpp \
    src/route_planner/route_planner.cpp \
    src/world_manager/world_manager.cpp \
    src/world_manager/world_committer.cpp \
    src/query_server/json_protocol.cpp \
    src/query_server/query_server.cpp

BUILD := build
OBJS  := $(patsubst %.cpp,$(BUILD)/%.o,$(WQS_SRC) $(RECAST_SRC) $(DETOUR_SRC))

.PHONY: all clean test service builder components audit

all: $(BUILD)/pathandreas $(BUILD)/navmesh_builder $(BUILD)/pathandreas_tests $(BUILD)/navmesh_components $(BUILD)/route_audit

$(BUILD)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD)/pathandreas: $(OBJS) src/main.cpp
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) src/main.cpp $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/navmesh_builder: $(OBJS) tools/navmesh_builder_cli.cpp
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tools/navmesh_builder_cli.cpp $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/pathandreas_tests: $(OBJS) tests/test_all.cpp
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/test_all.cpp $(OBJS) $(LDFLAGS) -o $@

test: $(BUILD)/pathandreas_tests
	$(BUILD)/pathandreas_tests

service: $(BUILD)/pathandreas

builder: $(BUILD)/navmesh_builder

clean:
	rm -rf $(BUILD)

$(BUILD)/navmesh_components: $(OBJS) tools/navmesh_components.cpp
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tools/navmesh_components.cpp $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/route_audit: $(OBJS) tools/route_audit.cpp
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tools/route_audit.cpp $(OBJS) $(LDFLAGS) -o $@

# Route regression gate. Needs the local game data, so it is not part of `test`.
audit: $(BUILD)/route_audit
	$(BUILD)/route_audit --cadb data/ColAndreas.cadb --navmesh data/gta.navmesh \
	    --paths paths/Paths
