
find_package(OpenCV REQUIRED )

FetchContent_Declare(
  fmt_fc
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG        6.0.0
)

FetchContent_Declare(
  cli11_fc
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v1.8.0
)

option(YAML_CPP_BUILD_TESTS "BUILD YAML Tests" OFF)
FetchContent_Declare(
  yaml_cpp_fc
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG  9a36242 # rrelease 0.6.3
)

FetchContent_MakeAvailable(fmt_fc cli11_fc yaml_cpp_fc)