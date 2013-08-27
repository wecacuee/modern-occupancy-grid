cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_policy(VERSION 2.8)
cmake_policy(SET CMP0015 NEW)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_CXX_FLAGS_SYNTAX "-x c++ -fsyntax-only")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Werror -O0 -ggdb")
set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_RELEASE} -lprofiler")

###################################################################################
# Find dependencies via installed packages
find_package(GTSAM REQUIRED NO_MODULE) 
include_directories(${GTSAM_INCLUDE_DIR})

find_package( OpenCV 2.4 REQUIRED core imgproc highgui)
include_directories(${OpenCV_INCLUDE_DIR})

###################################################################################
# Brian's code
# run in toplevel dir, for example "bin/SICKSlowMetropolis 18 18 0.18"
# after that, DisplayMarginals should display ?? But source is missing
include_directories(include/)

add_library(occupancy_grid_graph SHARED
			src/OccupancyGrid.cpp
			src/LaserFactor.cpp
			src/cvmat_serialization.cpp
			src/loadData.cpp
			src/utility.cpp
			)
target_link_libraries (occupancy_grid_graph gtsam-shared ${OpenCV_LIBS})

add_executable(SICKSlowMetropolis
	src/MCMC/Metropolis.cpp
	tests/testSICKSlowMetropolis.cpp
	)
target_link_libraries (SICKSlowMetropolis occupancy_grid_graph ${OpenCV_LIBS})

add_executable(SICK_DDMCMC
			src/MCMC/DDMCMC.cpp
			tests/testSICKDDMCMC.cpp
			)
target_link_libraries (SICK_DDMCMC occupancy_grid_graph gtsam-shared
			${OpenCV_LIBS})

###################################################################################
# Vikas' code with merali13icra implementation
# run anywhere, for example in Data/player_sim dir:
# ../../bin/inference_gibbs_sampling laser_pose_all.bin laser_range_all.bin scan_angles_all.bin
# ../../bin/two_assumption_alg laser_pose_all.bin laser_range_all.bin scan_angles_all.bin

find_package(Threads REQUIRED)

find_package( Boost 1.40 COMPONENTS date_time serialization-mt filesystem
	system REQUIRED)
include_directories( ${Boost_INCLUDE_DIR} )

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIR})

add_library(merali13icra SHARED
	src/occgrid.cpp
	src/forward_sensor_model.cpp
	src/cvmat_serialization.cpp)
target_link_libraries(merali13icra ${OpenCV_LIBS} ${Boost_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT} ${GTEST_BOTH_LIBRARIES})

add_executable(TwoAssumptionAlgorithm
	src/TwoAssumptionAlgorithm.cpp)
target_link_libraries(TwoAssumptionAlgorithm occupancy_grid_graph ${OpenCV_LIBS})

add_executable(simulateddata
	src/simulateddata.cpp)
target_link_libraries(simulateddata merali13icra 
	${OpenCV_LIBS} ${Boost_LIBRARIES})
add_executable(test_raytrace
	tests/test_raytrace.cpp)
target_link_libraries(test_raytrace 
	${CMAKE_THREAD_LIBS_INIT} ${GTEST_BOTH_LIBRARIES})

add_executable(dualdecomposition
	src/dualdecomposition.cpp)
target_link_libraries(dualdecomposition occupancy_grid_graph ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_slaveminimizer
	tests/test_slaveminimizer.cpp)
target_link_libraries(test_slaveminimizer occupancy_grid_graph ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(run_belief_propagation
	src/run_belief_propagation.cpp)
target_link_libraries(run_belief_propagation occupancy_grid_graph ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_utility
	src/utility.cpp
	tests/test_utility.cpp)
target_link_libraries(test_utility 
	${Boost_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT} ${GTEST_BOTH_LIBRARIES})

add_executable(test_sumproduct
	src/utility.cpp
	tests/test_sumproduct.cpp)
target_link_libraries(test_sumproduct 
	${Boost_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT} ${GTEST_BOTH_LIBRARIES})

add_executable(test_cartesian_product
	src/utility.cpp
	tests/test_cartesian_product.cpp)
target_link_libraries(test_cartesian_product 
	${Boost_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT} ${GTEST_BOTH_LIBRARIES})

add_executable(test_occgrid 
	src/forward_sensor_model.cpp
	tests/test_occgrid.cpp)
target_link_libraries(test_occgrid  merali13icra
	${CMAKE_THREAD_LIBS_INIT} ${GTEST_BOTH_LIBRARIES})

add_executable(inference_gibbs_sampling 
	src/inference_gibbs_sampling.cpp)
target_link_libraries(inference_gibbs_sampling merali13icra ${OpenCV_LIBS})

add_executable(visualize_ground_truth 
	src/visualize_ground_truth.cpp)
target_link_libraries(visualize_ground_truth merali13icra ${OpenCV_LIBS})

add_executable(two_assumption_alg 
	src/two_assumption_alg.cpp)
target_link_libraries(two_assumption_alg merali13icra ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(displayMarginals
	src/displayMarginals.cpp)
target_link_libraries(displayMarginals ${OpenCV_LIBS})

# find pkgconfig modules
include(FindPkgConfig)
pkg_search_module( PLAYER playerc++>=2.1.0 ) 
IF( PLAYER_FOUND )
  MESSAGE( STATUS ${INDENT} "Player version ${PLAYER_VERSION} detected at ${PLAYER_LIBRARIES}" )
  link_directories( ${PLAYER_LIBRARY_DIRS} )
  include_directories( ${PLAYER_INCLUDE_DIRS} )
  add_executable(captureplayerdata
	  src/captureplayerdata.cpp 
	  src/cvmat_serialization.cpp)
  target_link_libraries(captureplayerdata ${OpenCV_LIBS} ${PLAYER_LIBRARIES}
	  ${Boost_LIBRARIES})
ELSE( PLAYER_FOUND )
  MESSAGE( ${INDENT} "Player not detected. If Player is installed but not detected, check your PKG_CONFIG_PATH." )
ENDIF( PLAYER_FOUND )


