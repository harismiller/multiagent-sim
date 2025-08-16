# Multi-agent Planner Isaac Sim Visualization
This repository contains the files required to visualize the drone simulation example for a multi-agent version of the LTL-D* algorithm.
Link to paper: https://doi.org/10.48550/arXiv.2404.01219 (Citation to be added)

This simulation was built with Isaac Sim 4.2.0 and ROS2 Humble Hawksbill on Ubuntu 22.04 Jammy Jellyfish.

## Dependencies
* ROS2
* Isaac Sim 4.2.0 and higher.
* [interfaces_hmm_sim](https://github.com/harismiller/interfaces_hmm_sim)
* LTL-D* PLanner.

## Usage
### ROS2 Setup
Isaac Sim uses it's own compiled version of ROS2 Humble through FastDDS. The method for setting up the ROS2 bridge for Isaac Sim can be found here: [ROS2 Bridge Setup](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#isaac-sim-app-enable-ros). The `interfaces_hmm_sim` package also needs to be built in a ROS2 workspace. Place this folder in the `src` folder of a ROS2 Workspace and use the command `colcon build --packages-select interfaces_hmm_sim` to build it. Make sure to source both ROS2 Humble and this workspace in any terminal used to run the standalone example.

You can find the `interfaces_hmm_sim` package here: [interfaces_hmm_sim repo](https://github.com/harismiller/interfaces_hmm_sim).

### Isaac Sim Standalone Example
Place all other files in the same directory and run as an Isaac Sim standalone example. This is done by running the python.sh file for the version of Isaac Sim being used. Assuming Isaac Sim is installed to the default directory on Linux, it is recommended to add the following alias to your .bashrc or other shell environment file to make running standalone environments easier.
```
alias isaac_python='/home/<USER>/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh'
```
Make sure to replace `<USER>` with your own system user or the whole path if Isaac Sim was installed to a different location. From here navigate to the directory where `standalone-multiagent.py` is located and run the following command to run the simulation.
```
isaac_python isaac-multiagent-halton.py -<necessary flags>
```
If you did not create the above alias, you will instead have to use the full path to the python.sh file to run the example. 

**Note: It can take a while to load the full environment, especially if it is quite large. Unless the terminal says that Isaac Sim has crashed, it is recommended to wait a bit for Isaac Sim to load, even if says that it is not responding. The example environment provided takes ~1 to 5 minutes to load depending on the PC specs used to load it. If it takes longer than 10 minutes to load, there may be a problem with the environment USD file or standalone example code.** 

### Simulation Flags
In order to run the simulation, you will need to setup some initial conditions. Running `isaac_python isaac-multiagent-halton.py -h` will bring up a list of options for how running the standalone example. The easiest method for setting up the simulation is to use a setup.yaml file. Here is an example of what a setup.yaml file might look like.
```
files:
    - halton: halton_points.csv
    - environment: Halton_env.usd
setup:
    - num_quads: 2
    - num_turtle: 2
    - environment_scale: 4.0
agents:
    - id: 1
      type: quad
      initial_position: [0, 0, 0]
    - id: 2
      type: turtle
      initial_position: [1, 1, 0]
    - id: 3
      type: turtle
      initial_position: [2, 2, 0]
    - id: 4
      type: quad
      initial_position: [3, 3, 0]
```

Please make sure that all quadcopters and turtlebots exists inside the environment USD as this is how this simple example finds the agents in the world to move. The initial position in the setup file should not be based on the environment grid positions, but instead the initial position as described in the planner. See the `Halton_env.usd` environment as an example for how to setup the agents. At the moment, this simulation only supports square environments for scaling with the planner.

Make sure that the environment USD file is placed in the `environments` directory. **Note that this is not the same as the env directory which contains the `halton_points.csv` file.**

Once the setup file is created simply place it in the same directory as `isaac-multiagent-halton.py` and run the following command.
```
isaac_python isaac-multiagent-halton.py --setup_file <SETUP_FILE_NAME>.yaml
```
Replace `<SETUP_FILE_NAME>` with the name of your setup file.

The environment USD file referenced in this example is too large to be uploaded to GitHub and can be found at the following link: [Environment USDs](https://gtvault-my.sharepoint.com/:f:/g/personal/hmiller43_gatech_edu/EiXSVUZyHghKi__9jqqyTwMBl0gtiv0T9s5Oy3fTkMTuDQ?e=raiM9h)

### Controls
The following keyboard inputs can be used once the simulation window has been fully loaded:
* P - Start/Stop the sim
* X - Close the sim

## Contact
For any questions or comments, please contact  Haris Miller (hhmille@sandia.gov).
