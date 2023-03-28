Test
```
ctest -R cxy_test
```

The following build process is tested on a clean Ubuntu 20.04 system.

Install dependent libraries
```   
sudo apt-get install build-essential cmake pkg-config git

sudo apt-get install libglpk-dev libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev libopenscenegraph-dev libbullet-dev libtinyxml2-dev liburdfdom-dev libxi-dev libxmu-dev freeglut3-dev
```

install qhull

```
git clone https://github.com/qhull/qhull.git
cd qhull
cd build
cmake ..
make
ctest
sudo make install
```

Install yaml-cpp
```
git clone https://github.com/jbder/yaml-cpp.git
cd yaml-cpp
cd build
cmake ..
make
sudo make install
```


Update the external libraries
```
git submodule update --init --recursive --progress
```

You have can build and install dart in the system or build dart in this project. 
If you want to build and install dart:
```
cd mechanics/external/dartsim
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

If you don't want to build and install dart, comment `FIND_PACKAGE(DART REQUIRED COMPONENTS gui collision-bullet utils)` in the ./CMakeLists.txt, and uncomment `add_subdirectory(dartsim)` in ./mechanics/external/CMakeLists.txt. And proceed to the next step. 

Use cmake to build this project in the project root folder
```
mkdir build
cd build
cmake ..
make
```

Check out examples in the build folder

```
...
```

Add `export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/lib"` to ~/.bashrc