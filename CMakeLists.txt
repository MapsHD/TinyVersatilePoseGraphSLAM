cmake_minimum_required(VERSION 3.15.0)

project(tiny-versatile-pose-graph-slam)

set (HDMAPPING_VERSION_MAJOR 0)
set (HDMAPPING_VERSION_MINOR 0)
set (HDMAPPING_VERSION_PATCH 0)

set(CMAKE_CXX_STANDARD 20)

add_definitions(-D_USE_MATH_DEFINES)

find_package(OpenGL REQUIRED)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

get_filename_component(REPOSITORY_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
set(EXTERNAL_LIBRARIES_DIRECTORY ${REPOSITORY_DIRECTORY}/3rdparty)

# Option to build with bundled Eigen
option(BUILD_WITH_BUNDLED_EIGEN "Build with bundled Eigen" ON)
if (BUILD_WITH_BUNDLED_EIGEN)
    set(EIGEN_BUILD_DOC
            OFF
            CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_TESTING
            OFF
            CACHE BOOL "" FORCE)

    add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/eigen)
    set(EIGEN3_INCLUDE_DIR ${EXTERNAL_LIBRARIES_DIRECTORY}/eigen)
    MESSAGE(STATUS "Using bundled Eigen3 : ${EIGEN_INCLUDE_DIR}")
else()
    find_package(Eigen3 REQUIRED)
    MESSAGE(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
endif()

option(BUILD_WITH_BUNDLED_FREEGLUT "Build with bundled FreeGlut" ON)
if (BUILD_WITH_BUNDLED_FREEGLUT)
    set(FREEGLUT_BUILD_DEMOS
            OFF
            CACHE BOOL "" FORCE)
    add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut)
    set(FREEGLUT_INCLUDE_DIR ${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut/include)
    set(FREEGLUT_LIBRARY freeglut)
    MESSAGE(STATUS "Using bundled FreeGlut")
else()
    find_package(GLUT REQUIRED)
    set(FREEGLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIR})
    set(FREEGLUT_LIBRARY ${GLUT_glut_LIBRARY})
    MESSAGE(STATUS "FreeGlut include dir: ${FREEGLUT_INCLUDE_DIR}")
endif ()

add_executable(
        example examples/example.cpp 
        src/TinyVersatilePoseGraphSLAM.cpp 
        src/TinyVersatilePoseGraphSLAM_AtPA_AtPb_TaitBryan.cpp
        src/TinyVersatilePoseGraphSLAM_AtPA_AtPb_Rodrigues.cpp
	src/TinyVersatilePoseGraphSLAM_AtPA_AtPb_Quaternion.cpp
	src/TinyVersatilePoseGraphSLAM_uncertainty_calculator.cpp
)
target_link_libraries(
  example
  PRIVATE
          ${FREEGLUT_LIBRARY}
          ${OPENGL_gl_LIBRARY}
          OpenGL::GLU)

target_include_directories(
  example
  PRIVATE include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-2.2.0/include
          ${FREEGLUT_INCLUDE_DIR})

if(WIN32)
  add_custom_command(
    TARGET example
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:example>
            $<TARGET_FILE_DIR:example>
    COMMAND_EXPAND_LISTS)
endif()

install(TARGETS example DESTINATION bin)
