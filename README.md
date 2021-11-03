# Design Model Language IDE
**DML** is an IDE for modeling, simulation, and optimizing physical structures. DML stands for **Design Model Language**, which is the underlying format read by the IDE software. DML is an xml-like schema that allows users to define designs and simulations.

The DML-IDE utilizes [Titan](https://github.com/swyetzner/Titan) as its underlying physics engine. Titan provides a GPU-parallelized simulation library for near-real-time interaction.

## Requirements
* A C++11 compiler
* [CMake](https://cmake.org/)
* At least one CUDA-compataible GPU. See [here](https://developer.nvidia.com/cuda-gpus) for a list.

## Build
DML-IDE has the following dependencies. Download and setup them up individually from the links below *OR* use a package manager (for everything but Qt and Lib3MF); [vcpkg](https://docs.microsoft.com/en-us/cpp/build/vcpkg?view=vs-2019) is highly recommended.
* [Qt](https://www.qt.io/download)
* [CUDA](https://developer.nvidia.com/cuda-toolkit)
* [GLEW](http://glew.sourceforge.net/)
* [GLFW](https://www.glfw.org/)
* [glm](https://github.com/g-truc/glm)
* [args](https://taywee.github.io/args/)
* [pugixml](https://github.com/zeux/pugixml)
* [Eigen](http://eigen.tuxfamily.org)
* [Spectra](https://github.com/yixuan/spectra)
* [Lib3MF](https://github.com/3MFConsortium/lib3mf)

For installing Lib3MF first clone the repository from [Lib3MF](https://github.com/3MFConsortium/lib3mf). Download along with submodule and move into the repository.
````
$ git clone --recurse-submodules https://github.com/3MFConsortium/lib3mf
$ cd lib3mf
````
Call GenerateMake.sh (Linux, Mac) or define other targets by specifying custom CMake generators (default is best unless you know what you're doing).
A new folder “build” is created and contains projects for the IDE/build target you selected.
For Linux/Mac:
  * navigate to the “build”-folder
  * Call “make” to build the projects
  * Run/debug a project and install the library in the default location
````
$ cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX
$ make
$ make tests
$ make install
````
Lib3MF should now be ready to go. 

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
© 2020 The Trustees of Columbia University in the City of New York. This work may be reproduced and distributed for academic non-commercial purposes only without further authorization, but rightsholder otherwise reserves all rights and all reproductions of this work must include an express acknowledgment of Sofia Wyetzner and Professor Hod Lipson of the Creative Machines Lab at Columbia University as the source. No changes or modifications to this work may be made without prior written permission from Sofia Wyetnzer and Professor Hod Lipson.
