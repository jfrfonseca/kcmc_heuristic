cmake_minimum_required(VERSION 3.13)
project(kcmc_heuristic)

set(CMAKE_CXX_STANDARD 14)

# Utilities and KCMC Instance Object ------------------------------------------
ADD_LIBRARY(KCMC_Module
            src/kcmc_instance.cpp
            src/k_coverage.cpp
            src/m_connectivity_greedy.cpp
            src/heuristic_greedy.cpp
            src/m_connectivity_dinitz.cpp
            src/heuristic_dinitz.cpp
            src/kcmc_instance.h
)
target_link_libraries(KCMC_Module)


# Instance generator ----------------------------------------------------------
ADD_EXECUTABLE(instance_generator src/instance_generator.cpp)
target_link_libraries(instance_generator KCMC_Module)


# Instance validator ----------------------------------------------------------
ADD_EXECUTABLE(instance_validator src/instance_validator.cpp)
target_link_libraries(instance_validator KCMC_Module)

ADD_EXECUTABLE(isomorphism_validator src/isomorphism_validator.cpp)
target_link_libraries(isomorphism_validator KCMC_Module)

ADD_EXECUTABLE(instance_visualizer src/instance_visualizer.cpp)
target_link_libraries(instance_visualizer KCMC_Module)


# Optimizers (DINITZ) --------------------------------------------------------

ADD_EXECUTABLE(heuristics src/heuristics_runtime.cpp)
target_link_libraries(heuristics KCMC_Module)
