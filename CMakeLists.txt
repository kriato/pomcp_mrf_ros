CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(pomcp_ros)

# SET(CMAKE_CXX_FLAGS " -g -O0") #-DCMAKE_BUILD_TYPE=RelWithDebInfo Debug
SET(CMAKE_CXX_FLAGS " -O3") #-DCMAKE_BUILD_TYPE=Release

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(Eigen3 REQUIRED)
FIND_PACKAGE(YAML REQUIRED)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
    roscpp
    tf_conversions
    move_base_msgs
    eigen_conversions
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp
    tf_conversions
    move_base_msgs
    eigen_conversions
)

SET(
    POMCP_SOURCES
    src/beliefstate.cpp
    src/coord.cpp
    src/experiment.cpp
    src/mcts.cpp
    src/node.cpp
    src/ros_rocksample.cpp
    src/simulator.cpp
    src/start_pomcp.cpp
    src/utils.cpp
)

SET(
    AGENT_SOURCES
    src/start_agent.cpp
    src/agent.cpp
)

SET(
    ENV_SOURCES
    src/start_env.cpp
    src/env.cpp
)

ADD_EXECUTABLE(start_pomcp ${POMCP_SOURCES})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(start_pomcp ${catkin_LIBRARIES} Eigen3::Eigen)

ADD_EXECUTABLE(start_agent ${AGENT_SOURCES})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(start_agent ${catkin_LIBRARIES})

ADD_EXECUTABLE(start_env ${ENV_SOURCES})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${YAML_INCLUDES})
TARGET_LINK_LIBRARIES(start_env ${catkin_LIBRARIES} ${YAML_LIBRARIES})