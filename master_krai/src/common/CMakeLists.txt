set(LIBRARY_NAME common)

add_library(${LIBRARY_NAME}
  SHARED
  math_func.cpp
  )

target_include_directories(${LIBRARY_NAME}
  PUBLIC ${PROJECT_SOURCE_DIR}/src/common
  )

target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})