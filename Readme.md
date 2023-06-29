# Installation

The following build process is tested on a clean Ubuntu 20.04 system.

Install dependent libraries
```   
sudo apt-get install build-essential cmake pkg-config git

sudo apt-get install libglpk-dev libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev libopenscenegraph-dev libbullet-dev libtinyxml2-dev liburdfdom-dev libxi-dev libxmu-dev freeglut3-dev

sudo apt-get install libnlopt-dev
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
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build
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
cd ./src/mechanics/external/dartsim
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

If you have error in finding nlopt, please install nlopt from source files
```
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```
And then go back to rebuild dart and this probject.

# Run Examples
The setup for different tasks is in `/data/{task-name}/setup.yaml`

To check a task
```
build/bin/hidex_planner data/{task-name}
```
Currently, the default is to visualize a plan it has generated and saved before. You can change the `visualize_option` in the setup.yaml in the task folder to do the planning. It includes "csv" (visualize a plan), "setup" (visualize the environment setup and start and goal), "save" (run the planner and save the result), "show" (run the planner, save the result, and visualize the plan). Please check the `setup.yaml` in `data/template_task` for more instructions.


Here is an example. Run this in the project root folder
```
build/bin/hidex_planner data/bookshelf
```
You will see the plan of pulling a book out of the bookshelf.


# Write your own task

1. You first want to create a task folder, it doesn't matter where you put the folder, but we recommend to put it under `{project-root}/data`. We recommend copying the folder `{project-root}/data/template_task`.

2. For starting, we recommend using our preset robot models. Then, the only thing you need to do is to modify the `setup.yaml`. Please follow the instructions in `{project-root}/data/template_task/setup.yaml` to make your own task. It is also beneficial to refer to other tasks in the `data` folder.

3. Now you are ready to run your own task with the command `build/bin/hidex_planner data/{your-task-folder}`

4. If you want to use your own robot models, it requires changes to the source code. 
