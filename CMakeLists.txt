cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
project(DMLIDE VERSION 0.1 LANGUAGES CXX CUDA)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
find_package(Qt5Xml CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(GLEW CONFIG REQUIRED)


set(OpenGL_GL_PREFERENCE "GLVND")

add_subdirectory(lib/Titan)
include_directories(lib/Titan/include)

remove_definitions(Titan PUBLIC GRAPHICS)

message(WARNING ${COMPILE_DEFINITIONS})

set(SOURCES
        src/window.ui
        src/resources.qrc
        src/window.h
        src/designViewer.h
        src/dmlTree.h
        src/loader.h
        src/model.h
        src/modelviewer.h
        src/properties.h
        src/simulator.h
        src/gUtils.h
        src/utils.h
        src/main.cpp
        src/window.cpp
        src/designViewer.cpp
        src/dmlTree.cpp
        src/loader.cpp
        src/modelviewer.cpp
        src/properties.cpp
        src/simulator.cpp
        src/gUtils.cpp
        src/utils.cpp
        src/polygonizer.cpp
        src/polygonizer.h
        src/polygon.cpp
        src/polygon.h
        src/exportThread.cpp
        src/exportThread.h
        src/optimizer.cpp
        src/optimizer.h
        src/commandLine.cpp
        src/commandLine.h)

set(CU_SOURCES
        src/simulatorParallel.h
        src/simulatorParallel.cu
        src/model.cu
)

set (QT_RESOURCES src/resources.qrc)

add_executable(${PROJECT_NAME} ${SOURCES} ${CU_SOURCES} ${QT_RESOURCES})

# Enable OpenMP compiler options if available
option (USE_OpenMP "Use OpenMP" ON)
if (USE_OpenMP)
        find_package(OpenMP)
        if (OpenMP_CXX_FOUND)
                SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
                SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        endif()
endif()

# Workaround for Qt/CUDA fPIC bug
if ( TARGET Qt5::Core )
        get_property( core_options TARGET Qt5::Core PROPERTY INTERFACE_COMPILE_OPTIONS )
        string( REPLACE "-fPIC" "" new_core_options "${core_options}" )
        set_property( TARGET Qt5::Core PROPERTY INTERFACE_COMPILE_OPTIONS ${new_core_options} )
        set_property( TARGET Qt5::Core PROPERTY INTERFACE_POSITION_INDEPENDENT_CODE "ON" )
        set( CMAKE_CXX_COMPILE_OPTIONS_PIE "-fPIC" )
endif()
set_source_files_properties(${CU_SOURCES} PROPERTIES COMPILE_FLAGS "-Xcompiler=-fPIC")
set_source_files_properties(${SOURCES} PROPERTIES COMPILE_FLAGS " -fPIC")


target_include_directories(${PROJECT_NAME} PUBLIC inc)

target_link_libraries(${PROJECT_NAME}
    PRIVATE Qt5::Widgets
    PRIVATE Qt5::Gui
    PRIVATE Qt5::Xml
    PRIVATE Titan
    PRIVATE glm
    PRIVATE ${GLEW_LIBRARIES}
)
