# sanitize environment before find_package, because otherwise it also looks in the directory created for the ExternalProject
include(EnvHelper)
sanitize_env()
find_package(Protobuf 3.3.0)
restore_env()

# protobuf versions >= 3.22 depend on abseil and require C++14+,
# which is incompatible with how the project is setup, so we build protobuf ourselves
if(NOT Protobuf_FOUND OR Protobuf_VERSION VERSION_GREATER_EQUAL 3.22)
  include(BuildProtobuf)
endif()
