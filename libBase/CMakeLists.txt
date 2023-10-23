include_directories(./include)
get_property(include_dir DIRECTORY ${CMAKE_CURRENT_LIST_DIR}  PROPERTY INCLUDE_DIRECTORIES)
set(BASE_INCLUDE ${include_dir} PARENT_SCOPE)

find_package(OpenCV 3 REQUIRED CONFIG)
include_directories(include ${OpenCV_INCLUDE_DIRS})

aux_source_directory(./src BASE_SRCS)
add_library(Base SHARED ${BASE_SRCS})
target_link_libraries(Base ${OpenCV_LIBS})
