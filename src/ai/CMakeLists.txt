file(GLOB_RECURSE SOURCES ./*.cpp)
add_library(lib-ai ${SOURCES})
target_link_libraries(lib-ai PUBLIC
  Boost::boost
  Boost::coroutine
  Boost::system
  Eigen3::Eigen
  Threads::Threads
  protobuf::libprotobuf
  ssl-protos::ssl-protos
)
target_include_directories(lib-ai PUBLIC ${PROJECT_SOURCE_DIR}/src)
set_target_properties(lib-ai PROPERTIES OUTPUT_NAME ai)
