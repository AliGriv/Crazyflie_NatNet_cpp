# Crazyflie with NatNet (Motive) in CXX
This repo is meant to provide necessary tools to operate Crazyflie drones in environments with Motive Motion Capture systems.
This work is based on [Crazyflie Toolbox](https://github.com/whoenig/crazyflie_tools) and it combined with
NatNet SDK provided to stream data to a Linux host. 

## How to build
I believe this is the way to run build the project :)

```
cd <repo_foldename>
mkdir build
cd build
cmake ..
make
``` 
## Before you use

### Single Copter Trajectory Tracking Flight
1. You have to define the trajectory. I should modify the code and make it more user friendly :) but I usually define it
all by defining `Trajectory_Planner::setPointCoordinates` in [Trajectory_PLanner](./src/Trajectory_Planner.h) header file.
2. Define the address to the crazyflie in [Main File](src/Main.cpp) where there is a list push it to the vector by
```
uri_list.emplace_back("radio://0/80/2M/E7E7E7E7E0");
```
3. Configure the Rigid Bodies on Motive software. Make sure you can stream rigid bodies (stream rigid bodies --> True).
4. build the project.
5. run `build/Main`


## Plotting the trajectories and errors and etc..
1. Go to the `build` directory: `cd build`
2. Make sure you have the requirements in [this file](src/requirements.txt).

*or*

Create a virtual environment
```
python3 -m venv env
source env/bin/activate
pip3 install -r requirements.txt
``` 
3. `python3 generatingPlotsFromCSV.py`