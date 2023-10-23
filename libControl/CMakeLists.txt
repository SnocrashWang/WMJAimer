add_subdirectory(Pose)
include_directories(${POSE_INCLUDE})
add_subdirectory(Aimer)
include_directories(${TOP_INCLUDE})

get_property(include_dir DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(CONTROL_INCLUDE ${include_dir} PARENT_SCOPE)
