cmake_minimum_required(VERSION 3.15)
project(CEVRP-with-Adaptive-Selection)

set(GOOGLETEST_VERSION 1.11.0)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3") # -O3 optimization argument

set(DEPENDENCIES
        src/heuristic.cpp
        include/heuristic.hpp
        src/stats.cpp
        include/stats.hpp
        src/utils.cpp
        include/utils.hpp
        src/individual.cpp
        include/individual.hpp
        src/case.cpp
        include/case.hpp
        src/MA.cpp
        include/MA.hpp
)

add_executable(Run main.cpp ${DEPENDENCIES})
target_link_libraries(Run PRIVATE pthread)

# Include directories
target_include_directories(Run PRIVATE src)

add_custom_target(valgrind
        COMMAND ${VALGRIND} --tool=memcheck --leak-check=full --show-leak-kinds=all ./Run
        DEPENDS Run
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
