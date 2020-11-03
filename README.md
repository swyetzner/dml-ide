# Design Model Language IDE
**DML** is an IDE for modeling, simulation, and optimizing physical structures. DML stands for **Design Model Language**, which is the underlying format read by the IDE software. DML is an xml-like schema that allows users to define designs and simulations.

The DML-IDE utilizes [Titan](https://github.com/swyetzner/Titan) as its underlying physics engine. Titan provides a GPU-parallelized simulation library for near-real-time interaction.

## Requirements
* A C++11 compiler
* [CMake](https://cmake.org/)
* At least one CUDA-compataible GPU. See [here](https://developer.nvidia.com/cuda-gpus) for a list.

## Build
DML-IDE has the following dependencies. Download and setup them up individually from the links below *OR* use a package manager (for everything but Qt); [vcpkg](https://docs.microsoft.com/en-us/cpp/build/vcpkg?view=vs-2019) is highly recommended.
* [Qt](https://www.qt.io/download)
* [CUDA](https://developer.nvidia.com/cuda-toolkit)
* [GLEW](http://glew.sourceforge.net/)
* [GLFW](https://www.glfw.org/)
* [glm](https://github.com/g-truc/glm)
* [args](https://taywee.github.io/args/)
* [pugixml](https://github.com/zeux/pugixml)
* [Eigen](http://eigen.tuxfamily.org)
* [Spectra](https://github.com/yixuan/spectra)


Download DML-IDE from the GitHub source along with its submodules. Once downloaded, move into the repository:
````
$ git clone --recurse-submodules https://github.com/swyetzner/dml-ide
$ cd dml-ide
````
We are now going to create a build directory and use CMake to build into it:
````
$ mkdir build
$ cmake ..
````
*Note:* You may need to append some flags onto the cmake command if it fails:
* Point CMake to your local copy of **Qt** with `-DCMAKE_PREFIX_PATH={path to Qt}/Qt/5.12.1/{architecture}/lib/cmake` where architecture would be gcc_64 on Linux, clang_64 on MacOS, etc. 
* If you are using **vcpkg** you need to add the flag `-DCMAKE_TOOLCHAIN_FILE={path to vcpkg}/vcpkg/scripts/buildsystems/vcpkg.cmake`.

We should now be able to make and run DMLIDE:
````
$ cd ..
$ make
$ ./DMLIDE
````

### License
This software was written by Sofia Wyetzner as part of a project led by Professor Hod Lipson at the Creative Machines Lab at Columbia University. You are welcome to use and modify the software as desired, but we ask that you give credit to the original source.
