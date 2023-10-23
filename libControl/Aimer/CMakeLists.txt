set(CMAKE_CXX_STANDARD 11)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
get_property(include_dir DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(WMJ_CONTROL_AIMER_INCLUDE ${include_dir} PARENT_SCOPE)

add_library(Aimer SHARED ./src/KinematicModel.cpp ./src/ModelObserver.cpp ./src/MPC.cpp ./src/Aimer.cpp)
target_link_libraries(Aimer AngleSolver Base)

add_definitions(-DAIM_CFG="${CMAKE_CURRENT_LIST_DIR}/Aimer.yaml")
