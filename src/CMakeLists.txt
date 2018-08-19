add_subdirectory(ai)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME}
  lib-ai
  ssl-protos
  ${Boost_COROUTINE_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
  ${Gtkmm3_LIBRARIES}
  ${Protobuf_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
)
