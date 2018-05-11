add_subdirectory(ai)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} lib-ai)
