# Fourier GR1 Simulation Scripts
This ROS ws that also contains python script files to start up a Isaac Sim stage. As I could not get the ROS code to work, I would advise simply copying the `docker` and `isaac` folders and rewriting the rest of the ROS code, maybe setting up the ROS ws again. The ROS policy runner example was taken from [Isaac Sim Git Repo](https://github.com/isaac-sim/IsaacSim-ros_workspaces), and is part of [this tutorial](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_rl_controller.html). 

## General information about the Isaac Sim scripts
The isaac sim scripts are likely the most useful part of this repo. These have been written specifically for Isaac Sim version 5.1.0. Isaac published a [migration guide](https://isaac-sim.github.io/IsaacLab/main/source/refs/migration.html), the primary change is the names of the libraries. 

The important files of this project are:
- The script `gr1_standalone.py` was a copy of `h1_standalone.py` (I have saved a copy in the folder `/copy`) but I added some code from `load_stage.py` to load a specific USDA file. This file loads a stage, loads an instance of the policy class `GR1Policy` that is stored in `gr1_policy.py`, and steps through the sim. It currently loads the USDA file `/assets/gr1_scene.usda`.
- The file `gr1_policy.py` has a class for the fourier GR1 policy, it loads in the policy file `fourier.pt` and the rl environment file `fourier.yaml`. The policy file is created in the `exported` folder of your specific network in `gr1_manip` after running `play.py`. The env file is `env.yaml` in the regular folder of your network, it is the same after training and playing.

## Running instructions
Run all commands in your isaac simulation repo, in the docker this should be `/isaac-sim/`
### Running the Simulation
```bash
python3 /fourier/ros_ws/isaac/gr1_standalone.py
```
This command will start the simulation and open the GUI. However, it will be different if you move this folder at all. Instead of using `/fourier/ros_ws/isaac/gr1_standalone.py`, you should use the absolute path to the file. You can find the absolute path to a directory by running `pwd` in terminal.

## Docker instructions
I have written a simple `Dockerfile` and a bash file to create and enter the docker. Similar to `gr1_manip` you can enter the container in VS code by clicking `ctrl+shift+p` and then clicking *Attach to running container* (The docker requires the folder `fourier-sim` to be in the same place, can be altered by changing the mounting in the command in `docker.sh`).

To start the docker container, in the folder `gr1_ros_workspace/docker` run:
```bash
python3 docker.sh
```
