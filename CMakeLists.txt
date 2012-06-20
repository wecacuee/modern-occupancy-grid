cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_policy(VERSION 2.8)
cmake_policy(SET CMP0015 NEW)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#definitions for gtsam
include_directories(../gtsam/gtsam)
link_directories(../gtsam/build/gtsam)

add_executable(SICKSlowMetropolis
			OccupancyGrid/src/OccupancyGrid.cpp
			OccupancyGrid/src/LaserFactor.cpp
			OccupancyGrid/src/MCMC/Metropolis.cpp
			tests/testSICKSlowMetropolis.cpp
			)
target_link_libraries (SICKSlowMetropolis gtsam)

add_executable(SICK_DDMCMC
			OccupancyGrid/src/OccupancyGrid.cpp
			OccupancyGrid/src/LaserFactor.cpp
			OccupancyGrid/src/MCMC/DDMCMC.cpp
			tests/testSICKDDMCMC.cpp
			)
target_link_libraries (SICK_DDMCMC gtsam)