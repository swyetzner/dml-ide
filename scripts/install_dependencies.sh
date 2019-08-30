#!/usr/bin/env bash
# This script defines functions that will install the DML-IDE dependencies
# Specifically meant for TACC systems (https://www.tacc.utexas.edu/)

export_shell_variables() {
    export LD_LIBRARY_PATH=$WORK/bin/lib:$WORK/bin/lib64:$LD_LIBRARY_PATH
    export LIBRARY_PATH=$WORK/bin/lib:$WORK/bin/lib64:$LIBRARY_PATH
    export INSTALL_PREFIX=$WORK/bin
    export PATH=$INSTALL_PREFIX/bin:$PATH
    export CMAKE_PREFIX_PATH=$INSTALL_PREFIX:$CMAKE_PREFIX_PATH
}

install_all_vcpkg() {
    cd $WORK
    if [ ! -d "vcpkg" ] ; then
        git clone https://github.com/Microsoft/vcpkg.git
    fi
    cd vcpkg
    ./bootstrap-vcpkg.sh

    ./vcpkg install glm
    ./vcpkg install glew
    ./vcpkg install glfw3
    ./vcpkg install args
    ./vcpkg install pugixml
}

install_ffmpeg() {
    _wget https://ffmpeg.org/releases/ffmpeg-4.1.3.tar.bz2
    cd ffmepg-4.1.3
    ./configure --disable-x86asm
    make
    make install DESTDIR=$INSTALL_PREFIX
    cd ..
    echo "Installed ffmpeg"
}

install_glew() {
    export GLEW_DEST=$INSTALL_PREFIX
    _wget https://sourceforge.net/projects/glew/files/glew/2.1.0/glew-2.1.0.tgz
    cd glew-2.1.0
    make
    make install
    cd ..
    echo "Installed GLEW"
}

install_glfw() {
    _get_github https://github.com/glfw/glfw.git
    cd glfw/build
    cmake -DCMAKE_C_FLAGS="-std=c99" -DGLFW_BUILD_EXAMPLES=false -DGLFW_BUILD_TESTS=false ..
    make
    make install
    cd ../..
    echo "Installed GLFW"
}

install_glm() {
    _install_github_via_cmake https://github.com/g-truc/glm.git
    echo "Installed GLM"
}

install_args() {
    _get_github https://github.com/Taywee/args.git
    cd args
    make
    make install DESTDIR=$INSTALL_PREFIX
    cd ..
    echo "Installed args"
}

install_ffmpeg() {
    _wget https://ffmpeg.org/releases/ffmpeg-4.1.3.tar.bz2
    cd ffmepg-4.1.3
    ./configure --disable-x86asm
    make
    make install DESTDIR=$INSTALL_PREFIX
    cd ..
    echo "Installed ffmpeg"
}

install_pugixml() {
  _wget https://github.com/zeux/pugixml/releases/download/v1.9/pugixml-1.9.tar.gz
  echo "Install pugixml"
}

_wget() {
    # set -e
    set -u
    local url="$1"
    local dirname="`basename $1`"
    if [ ! -e "$dirname" ] ; then
        echo $dirname
        wget -O $dirname $url
        case "${url##*.}" in
            "zip")
                echo "Unzipping zip format"
                unzip $dirname && rm $dirname
                ;;
            "tgz")
                echo "Unzipping tgz format"
                tar -zxf $dirname && rm $dirname
                ;;
            "bz2")
                echo "Unzipping bz2 format"
                tar -xjf $dirname && rm $dirname
                ;;
            "gz")
                echo "Unzipping tar.gz format"
                tar -xzf $dirname && rm $dirname
                ;;
            "xz")
                echo "Unzipping tar.xz format"
                tar -xvf $dirname && rm $dirname
                ;;
            *)
                echo "Do not know how to extract $dirname"
                ;;
        esac
    fi
    set +u
}

_get_github() {
    # set -e
    set -u
    local github_url="$1"
    local dirname="`basename $1`"
    local dirname="${dirname%.git}"
    if [ ! -d "$dirname" ] ; then
        git clone $github_url
        if [ ! -d "$dirname/build" ] ; then
            mkdir $dirname/build
        fi
    fi
    set +u
}

_install_github_via_cmake() {
    # set -e
    set -u
    local github_url="$1"
    local dirname="`basename $1`"
    local dirname="${dirname%.git}"
    if [ ! -d "$dirname" ] ; then
        git clone $github_url
        if [ ! -d "$dirname/build" ] ; then
            mkdir $dirname/build
        fi
    fi
    cd $dirname/build
    cmake -DCMAKE_INSTALL_PREFIX:PATH=$INSTALL_PREFIX -DCMAKE_BUILD_TYPE=Release ..
    make
    make install
    cd ..
    set +u
}
