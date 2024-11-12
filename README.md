# GP-Pipeline

## Setup
Either manually export environmental variables each session or add them to the .bashrc-file:

```export GPPP_CATKIN_WS_PATH="/Insert/Your/Path/Here"```


## Building Container

cd container
sudo apptainer build cirp.sif testing_pipeline.def
