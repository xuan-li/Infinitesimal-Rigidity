cmake_minimum_required(VERSION 3.14)
project(InfinitesimalRigidity)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake" CACHE STRING "Modules for CMake" FORCE)

include_directories(${CMAKE_SOURCE_DIR}/external/libigl/external/eigen external/libhedra/include)

########### libigl ##########

set(LIBIGL_ROOT ${CMAKE_SOURCE_DIR}/external/libigl)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
option(LIBIGL_WITH_PNG               "Use PNG"            ON)
option(LIBIGL_WITH_TETGEN            "Use TetGen"         ON)

find_package(LIBIGL REQUIRED)

find_package(OpenMesh REQUIRED)
include_directories(${OpenMesh_INCLUDE_DIRS})
add_definitions(-D_USE_MATH_DEFINES)

find_package(OpenVolumeMesh REQUIRED)
include_directories(${OPENVOLUMEMESH_INCLUDE_DIR})

include_directories(src/InfinitesimalDeformation)
include_directories(src/MeshStructure)
include_directories(src/MeshStructure/MeshSurface)
include_directories(src/MeshStructure/MeshVolume)
include_directories(src/Utilities)
include_directories(src/Viewer)

add_library(polypartition STATIC external/polypartition/src/polypartition.h external/polypartition/src/polypartition.cpp)
include_directories(external/polypartition/src)

add_executable(InfinitesimalRigidity
        src/InfinitesimalDeformation/AlgorithmCommon.h
        src/InfinitesimalDeformation/ControllerRigidity.h
        src/InfinitesimalDeformation/HEMatrixConstructor.h
        src/InfinitesimalDeformation/KernalSolver.h
        src/InfinitesimalDeformation/LinearSystemConstructor.h
        src/InfinitesimalDeformation/MotionSolver.h
        src/InfinitesimalDeformation/Preprocessor.h
        src/MeshStructure/MeshSurface/MeshSurface.h
        src/MeshStructure/MeshSurface/OpenMeshHeader.h
        src/MeshStructure/MeshVolume/MeshVolume.h
        src/MeshStructure/MeshVolume/OpenVolumeHeader.h
        src/Utilities/MergeMeshMatrix.h
        src/Utilities/SurfaceMatrixConverter.h
        src/Utilities/Tetrahedralize.h
        src/Utilities/Triangulate3DPolygon.h
        src/Utilities/VolumeMatrixConverter.h
        src/Viewer/Common.h
        src/Viewer/Controller.h
        src/Viewer/KeyBoard.h
        src/Viewer/Menu.h
        src/Viewer/ViewerData.h
        src/Viewer/ViewerFlags.h
        src/main.cpp)

target_link_libraries(InfinitesimalRigidity polypartition ${OPENVOLUMEMESH_LIBRARY} ${OpenMesh_LIBRARIES}
        igl::png igl::core igl::opengl igl::opengl_glfw igl::opengl_glfw_imgui
        igl::tetgen)
