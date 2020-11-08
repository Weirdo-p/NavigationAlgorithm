set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED true)

PROJECT(NavigationAlgorithm)
include_directories(include)
include_directories("/usr/include/eigen3")
AUX_SOURCE_DIRECTORY(src DIR_SRCS)
SET(TEST_MATH ${DIR_SRCS})
add_executable(navigation ${TEST_MATH})


