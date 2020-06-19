cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(CMAKE_PREFIX_PATH ~/Qt/5.14.2/gcc_64/lib/cmake)
set(CMAKE_TOOLCHAIN_FILE ~/Downloads/vcpkg/scripts/buildsystems/vcpkg.cmake)
project(DMLIDE VERSION 0.1 LANGUAGES CXX CUDA)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
find_package(Qt5Xml CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
message(STATUS "GLM included at ${GLM_INCLUDE_DIRS}")
include_directories(${GLM_INCLUDE_DIRS})
find_package(GLEW CONFIG)
find_package(pugixml CONFIG REQUIRED)

set(OpenGL_GL_PREFERENCE "GLVND")

add_subdirectory(lib/Titan)
include_directories(lib/Titan/include)

remove_definitions(Titan PUBLIC GRAPHICS)

message(WARNING ${COMPILE_DEFINITIONS})

set(SOURCES
		src/model.h
		src/loader.h
		src/simulator.h
		src/optimizer.h
		src/oUtils.h
		src/utils.h
		src/parser.h
		src/polygon.h
		src/polygonizer.h
		src/loader.cpp
		src/simulator.cpp
		src/optimizer.cpp
		src/oUtils.cpp
		src/utils.cpp
		src/parser.cpp
		src/polygon.cpp
		src/polygonizer.cpp
		src/main.cpp
)

set (IO_SOURCES 
		src/io/commandLine.h
		src/io/exportThread.h
		src/io/commandLine.cpp
		src/io/exportThread.cpp
)

set (QT_SOURCES 
		src/gui/window.h
		src/gui/dmlTree.h
		src/gui/properties.h
		src/gui/exportdialog.h
		src/gui/dmlTree.cpp
		src/gui/properties.cpp
		src/gui/exportdialog.cpp
		src/gui/exportdialog.ui
		src/gui/window.cpp
		src/gui/window.ui
)

set (GL_SOURCES 
		src/opengl/designViewer.h
		src/opengl/simViewer.h
		src/opengl/gUtils.h
		src/opengl/designViewer.cpp
		src/opengl/simViewer.cpp
		src/opengl/gUtils.cpp
)
include_directories(src)
set (QT_RESOURCES src/resources.qrc)

option (USE_OpenGL "Use OpenGL" ON)
if (USE_OpenGL)
	add_executable(${PROJECT_NAME} ${SOURCES} ${IO_SOURCES} ${QT_SOURCES} ${QT_RESOURCES} ${GL_SOURCES})
	target_compile_definitions(${PROJECT_NAME} PUBLIC USE_OpenGL)
else()
	add_executable(${PROJECT_NAME} ${SOURCES} ${IO_SOURCES} ${QT_SOURCES}	)
endif()

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

set_source_files_properties(${SOURCES} PROPERTIES COMPILE_FLAGS " -fPIC")

target_include_directories(${PROJECT_NAME} PUBLIC inc)


target_link_libraries(${PROJECT_NAME}
    PRIVATE Qt5::Widgets
    PRIVATE Qt5::Gui
    PRIVATE Qt5::Xml
    PRIVATE Titan
    PRIVATE glm
    PRIVATE GLEW::GLEW
    PRIVATE pugixml
)
