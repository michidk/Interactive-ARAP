set(BINARY ${CMAKE_PROJECT_NAME})

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
include(libigl)

# OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Add your project files
set(HEADER_FILES arap.h gui.h utils.h)
set(SOURCE_FILES arap.cpp gui.cpp main.cpp utils.cpp)

add_executable(${BINARY} ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${BINARY} PUBLIC igl::core igl::opengl_glfw)

# add_library(${BINARY}_lib STATIC ${SOURCE_FILES})
# target_include_directories(${BINARY}_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} igl::core igl::opengl_glfw)

# build tests
set(TEST_SOURCES arap.cpp utils.cpp tests.cpp)
set(TEST_HEADERS arap.h utils.h  catch.hpp)

add_executable(tests ${TEST_HEADERS} ${TEST_SOURCES})
target_link_libraries(tests
        PUBLIC
        igl::core
        igl::opengl_glfw
        )