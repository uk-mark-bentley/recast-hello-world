# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

execute_process(COMMAND powershell.exe -Command "Write-Output \$env:USERPROFILE" OUTPUT_VARIABLE WINDOWS_PROFILE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
MESSAGE(STATUS "********** WINDOWS_PROFILE_PATH: ${WINDOWS_PROFILE_PATH}")
execute_process(COMMAND wslpath -a "${WINDOWS_PROFILE_PATH}" OUTPUT_VARIABLE WINDOWS_PROFILE_PATH_WSL OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE WSL_RESULT)
IF(WSL_RESULT)
	MESSAGE(FATAL_ERROR "Failed to execute wslpath command: ${WSL_RESULT}")
ENDIF()
MESSAGE(STATUS "********** WINDOWS_PROFILE_PATH_WSL: ${WINDOWS_PROFILE_PATH_WSL}")

project ("NavMeshTest")

# Include sub-projects.
add_subdirectory ("NavMeshTest")