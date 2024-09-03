cmake_minimum_required(VERSION 3.10)
project(casadi_python_cpp)

# basic cmake variables
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -pedantic -fPIC -O0 -fsanitize=address -fno-omit-frame-pointer")
# set(CMAKE_C_FLAGS "-g -Wall -Wextra -pedantic -fPIC -O0 -fsanitize=address -fno-omit-frame-pointer")
# set(CMAKE_LINKER_FLAGS "-fsanitize=address -fuse-ld=ld")

# find casadi and ipopt
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} .venv/lib/python3.12/site-packages/casadi)
find_package(casadi REQUIRED)
find_library(IPOPT_LIBRARY NAMES ipopt PATHS .venv/lib/python3.12/site-packages/casadi)

# create executable
add_executable(casadi_python_cpp main.cpp gen.c)
target_link_libraries(casadi_python_cpp m casadi ${IPOPT_LIBRARY})
