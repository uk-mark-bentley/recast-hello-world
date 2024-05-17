# CMakeList.txt : CMake project for NavMeshTest, include source and define
# project specific logic here.
#
file(GLOB SOURCES src/*.cpp src/*.h)

# Create a variable to hold the path to Recast navigation repository
set(RECAST_NAVIGATION_DIR "${WINDOWS_PROFILE_PATH_WSL}/ioTrain3rdPartyDependencies/RecastNavigation/recastnavigation")
get_filename_component(RECAST_NAVIGATION_DIR "${RECAST_NAVIGATION_DIR}" ABSOLUTE)
message(STATUS "********** RECAST_NAVIGATION_DIR: ${RECAST_NAVIGATION_DIR}")





# TODO: Add tests and install targets if needed.
if(NOT SDL2_ROOT_DIR)
  if(WIN32)
    set(SDL2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Contrib/SDL")
  elseif(APPLE)
    set(SDL2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Bin")
  endif()
endif()

find_package(OpenGL REQUIRED)
if(APPLE)
  find_library(SDL2_LIBRARY 
    NAMES SDL2
    PATHS ${SDL2_ROOT_DIR}
    REQUIRED)
else()
  find_package(SDL2 REQUIRED)
endif()

include_directories(SYSTEM ${OPENGL_INCLUDE_DIR})
include_directories(${RECAST_NAVIGATION_DIR}/DebugUtils/Include)
include_directories(${RECAST_NAVIGATION_DIR}/Detour/Include)
include_directories(${RECAST_NAVIGATION_DIR}/DetourCrowd/Include)
include_directories(${RECAST_NAVIGATION_DIR}/DetourTileCache/Include)
include_directories(${RECAST_NAVIGATION_DIR}/Recast/Include)

file(GLOB DEBUG_UTILS_SRC "${RECAST_NAVIGATION_DIR}/DebugUtils/Source/*.cpp")
add_library(DebugUtils SHARED ${DEBUG_UTILS_SRC})

file(GLOB RECAST_SRC "${RECAST_NAVIGATION_DIR}/Recast/Source/*.cpp")
add_library(Recast SHARED ${RECAST_SRC})

file(GLOB DETOUR_SRC "${RECAST_NAVIGATION_DIR}/Detour/Source/*.cpp")
add_library(Detour SHARED ${DETOUR_SRC})

file(GLOB DETOUR_CROWD_SRC "${RECAST_NAVIGATION_DIR}/DetourCrowd/Source/*.cpp")
add_library(DetourCrowd SHARED ${DETOUR_CROWD_SRC})

file(GLOB DETOUR_TILE_CACHE_SRC "${RECAST_NAVIGATION_DIR}/DetourTileCache/Source/*.cpp")
add_library(DetourTileCache SHARED ${DETOUR_TILE_CACHE_SRC})

# Add source to this project's executable.
add_executable (NavMeshTest ${SOURCES})

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET NavMeshTest PROPERTY CXX_STANDARD 20)
endif()

target_link_libraries(NavMeshTest Recast Detour DebugUtils DetourCrowd DetourTileCache ${SDL2_LIBRARY} ${OPENGL_LIBRARIES})
