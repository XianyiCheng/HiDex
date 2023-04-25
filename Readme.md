Test
```
ctest -R cxy_test
```

# Installation

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

If you don't want to build and install dart, comment `FIND_PACKAGE(DART REQUIRED COMPONENTS gui collision-bullet utils)` in the ./CMakeLists.txt, and uncomment `add_subdirectory(dartsim)` in ./src/mechanics/external/CMakeLists.txt. And proceed to the next step. 

Use cmake to build this project in the project root folder
```
mkdir build
cd build
cmake ..
make
```

Add `export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/lib"` to ~/.bashrc


# Run Examples
The setup for different tasks is in `/data/{task-name}/setup.yaml`

To run the planner
```
build/bin/hidex_planner {path_to_setup.yaml}
```

Here is an example. Run this in the project root folder
```
build/bin/hidex_planner ./data/bookshelf/setup.yaml
```

You should change the `visualize_option` in the setup.yaml to tell the planner what you want to see.

# Write your own task

1. You first want to create a task folder, it doesn't matter where you put the folder, but we recommend you to put it under `{project-root}/data/`.

2. If you want to use your own robot models, it requires you to change the source code. Please jump to ???. For starting, we recommend using our preset robot models.

3. If you are okay to start with the preset robot models, the only thing you need to do is to modify the `setup.yaml`. Please copy the file `{project-root}/data/template_task/setup.yaml` to your task folder. Please carefully follow the instructions in the file to make your own task. 

5. Now you are ready to run your own task with the command `build/bin/hidex_planner ./data/{your-task-folder}/setup.yaml`