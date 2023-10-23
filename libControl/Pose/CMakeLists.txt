include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
get_property(include_dir DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(POSE_INCLUDE ${include_dir} PARENT_SCOPE)

add_definitions(-DANGLE_CFG="${CMAKE_CURRENT_SOURCE_DIR}/AngleSolver.yaml")
add_library(AngleSolver SHARED ./src/AngleSolver.cpp)
target_link_libraries(AngleSolver Base)
