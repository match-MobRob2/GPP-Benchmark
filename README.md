# GPP-Benchmark
GPP-Benchmark is a containerized benchmarking tool designed to evaluate global path planning algorithms in a standardized, transferable, and repeatable manner. Built on the ROS2 navigation stack and utilizing Apptainer for containerization, this framework ensures consistent testing across different platforms.

The tool provides automated data recording, streamlined visualization, and supports a variety of standardized metrics, allowing users to assess path length, energy efficiency, and smoothness based on their specific application needs. Whether optimizing for logistics, exploration, or other domains, GPP-Benchmark offers a reliable way to compare algorithms across diverse hardware setups.

## Base Setup
1. Create a workspace- and src-folder. In this example we will use the `cirp_ws` workspace folder in the home directory.
```
cd ~
mkdir cirp_ws
cd cirp_ws
mkdir src
```
2. Clone the repository into the `src`-folder
3. Build the container, see the chapter [Build Container](#build-container) for detailed information.
4. Execute the container, see the chapter [Execute Container](#execute-container) for detailed information.

**Important:** Either manually export environmental variables each session or add them to the .bashrc-file:
```export GPPP_CATKIN_WS_PATH="/Insert/Your/Path/Here"```


## Build Container

```
cd ~/cirp_ws/src/gpp_pipeline/container
sudo apptainer build cirp.sif testing_pipeline.def
```

## Execute Container
**Important:** Remember setting the `GPPP_CATKIN_WS_PATH` env-variable. (See [Base Setup](#base-setup) for more information)

```
cd ~/cirp_ws/src/gpp_pipeline/container
apptainer run cirp.sif container_mode
```

Options for the parameter `container_mode` are:
`start`: Starts the testing process with all configurations set for the number of times specified.
`data_compare`: Compare different path planners
`data_visualize`: Visualize a dataset
