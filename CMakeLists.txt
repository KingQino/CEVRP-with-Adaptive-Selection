cmake_minimum_required(VERSION 3.15)
project(CEVRP-with-Adaptive-Selection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3") # -O3 optimization argument

set(CORE_SOURCES
        src/parameters.cpp
        src/command_line.cpp
        src/stats.cpp
        src/initializer.cpp
        src/leader.cpp
        src/follower.cpp
        src/reproduction.cpp
        src/individual.cpp
        src/case.cpp
        src/MA.cpp
)

find_package(Threads REQUIRED)
include(CheckIPOSupported)
check_ipo_supported(RESULT IPO_SUPPORTED OUTPUT IPO_ERROR)

add_library(cevrp_core STATIC ${CORE_SOURCES})
target_include_directories(cevrp_core PUBLIC include)
target_link_libraries(cevrp_core PUBLIC Threads::Threads)

add_executable(Run main.cpp)
target_link_libraries(Run PRIVATE cevrp_core)

if(IPO_SUPPORTED)
    set_target_properties(cevrp_core Run PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(module_smoke_test tests/module_smoke_test.cpp)
    target_link_libraries(module_smoke_test PRIVATE cevrp_core)
    target_compile_definitions(module_smoke_test PRIVATE
            TEST_DATA_DIRECTORY="${CMAKE_SOURCE_DIR}/data")
    add_test(NAME module_smoke_test COMMAND module_smoke_test)

    add_executable(command_line_test tests/command_line_test.cpp)
    target_link_libraries(command_line_test PRIVATE cevrp_core)
    add_test(NAME command_line_test COMMAND command_line_test)
endif()

add_custom_target(valgrind
        COMMAND ${VALGRIND} --tool=memcheck --leak-check=full --show-leak-kinds=all ./Run
        DEPENDS Run
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
