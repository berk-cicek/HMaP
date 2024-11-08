![flowchart_new](https://github.com/user-attachments/assets/ea06998d-1b73-4e34-b727-68b0cfdfdb89)# H-MaP: Iterative and Hybrid Sequential Manipulation Planner
## Overview
<p align="center">
  <img src="https://github.com/berk-cicek/HMaP/blob/main/misc/Diagram.svg" alt="HMaP"/>
</p>!

This paper introduces H-MaP, a hybrid sequential manipulation planner that addresses complex tasks requiring both sequential actions and dynamic contact mode switches. Our approach reduces configuration space dimensionality by decoupling object trajectory planning from manipulation planning through object-based waypoint generation, informed contact sampling, and optimization-based motion planning. This architecture enables handling of challenging scenarios involving tool use, auxiliary object manipulation, and bimanual coordination. Experimental results across seven diverse tasks demonstrate H-MaP's superior performance compared to existing methods, particularly in highly constrained environments where traditional approaches fail due to local minima or scalability issues. The planner's effectiveness is validated through both simulation and real-robot experiments.
https://sites.google.com/view/h-map/

## Installation
### Requirments
Assumes a standard Ubuntu 20.04 (or 18.04) machine.
The following assumes $HOME/git as your git path, and $HOME/.local to install 3rd-party libs -- please stick to this (no system-wide installs).
```
sudo apt update
sudo apt install --yes \
  g++ clang make cmake curl git wget \
  liblapack-dev libf2c2-dev libqhull-dev libeigen3-dev libann-dev libccd-dev \
  libjsoncpp-dev libyaml-cpp-dev libpoco-dev libboost-system-dev portaudio19-dev libusb-1.0-0-dev libhidapi-dev \
  libx11-dev libglu1-mesa-dev libglfw3-dev libglew-dev freeglut3-dev libpng-dev libassimp-dev
mkdir -p $HOME/git $HOME/.local
```
External libraries: You can skip librealsense and libfranka if you disable in CMake.
```
export MAKEFLAGS="-j $(command nproc --ignore 2)"
wget https://github.com/MarcToussaint/rai-extern/raw/main/install.sh; chmod a+x install.sh
./install.sh fcl
./install.sh physx
./install.sh librealsense
./install.sh libfranka  ## for OLD frankas instead:   ./install.sh -v 0.7.1 libfranka
```
You can skip this, if you disable pybind11 in CMake.
```
sudo apt install --yes python3-dev python3 python3-pip
python3 -m pip install --user numpy pybind11 pybind11-stubgen

# add the following to your .bashrc, if not done already
echo 'export PATH="${PATH}:$HOME/.local/bin"' >> ~/.bashrc
echo 'export PYTHONPATH="${PYTHONPATH}:$HOME/.local/lib"' >> ~/.bashrc
```
### Quick Start
```
cd $HOME/git
```
#### Build the Botop submodule.
```
git clone --recurse-submodules git@github.com:MarcToussaint/botop.git
cd botop
export PY_VERSION=`python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))"`
cmake -DUSE_BULLET=OFF -DUSE_OPENCV=OFF -DPY_VERSION=$PY_VERSION . -B build  #options: disable USE_LIBFRANKA USE_REALSENSE USE_PYBIND USE_PHYSX
make -C build/
```
#### Build the HMaP examples.
```
cd ..
git clone https://github.com/berk-cicek/HMaP
cd HMaP/test/<test_name>/
make
```

## Example Usage
<div align="center">
  <img src="https://github.com/berk-cicek/HMaP/blob/main/misc/bolt.gif" width="250" height="250" /> 
  <img src="https://github.com/berk-cicek/HMaP/blob/main/misc/tunnel.gif" width="250" height="250" />
</div>

<p align="center">
  <img src="https://github.com/berk-cicek/HMaP/blob/main/misc/Experiments.png" alt="HMaP1"/>
</p>

```
cd test/<test_name>/
./x.exe
```
To understand how HMaP operates, edit the environment in src/config and task parameters in src/HMAPBiman.cpp.
To understand how informed contact sampling operates, you can navigate to its directory, 'informedContact' to take a closer look.
<p align="center">
  <img src="https://github.com/berk-cicek/HMaP/blob/main/misc/Realrobot.JPG" alt="HMaP2"/>
</p>
