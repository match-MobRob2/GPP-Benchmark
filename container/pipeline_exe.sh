# Print a nice match-Logo
echo "                                                                                
                                        @@@@                   @@@@@          
                                        @@@@                   @@@@@          
@@@@@@@@@@@@@@@@@@@   @@@@@@@@@@@@@  @@@@@@@@@@@   @@@@@@@@@@@ @@@@@@@@@@@@@@ 
@@@@   @@@@@   @@@@@           @@@@@    @@@@      @@@@@        @@@@@     @@@@ 
@@@@   @@@@@    @@@@  @@@@@@@@@@@@@@    @@@@     @@@@@         @@@@@     @@@@ 
@@@@   @@@@@    @@@@ @@@@       @@@@    @@@@      @@@@         @@@@@     @@@@ 
@@@@   @@@@@    @@@@  @@@@@@@@@@@@@@    @@@@@@@@   @@@@@@@@@@@ @@@@@     @@@@ 
                                                                            "
                                                                            
# Source ros enviroment
echo "Sourcing environment now..."

if [ -f /opt/ros/humble/setup.bash ]
then
    source /opt/ros/humble/setup.bash
fi

echo "1"

if [ -f /cirp_ws/install/setup.bash ]
then
    source /cirp_ws/install/setup.bash
fi

echo "2"

# This setting is absolutly mandatory for the cluster!
# The cluster is not blocking or ROS messages from one user to the other
# If this is not set other users can see my topics and also the simulations mess each other up (time jump detected)
# This prevents it in a nices way than running "singularity run --net --network none match-dt-tasks.sif XX"
export ROS_LOCALHOST_ONLY=1

# if [ $1 == "rebuild" ] # Remove existing built components and build all packages in tasks_ws workspace
# then
#     rm -rf ${SINGULARITY_ROOTFS}/tasks_ws/install
#     rm -rf ${SINGULARITY_ROOTFS}/tasks_ws/build
#     rm -rf ${SINGULARITY_ROOTFS}/tasks_ws/log
#     cd ${SINGULARITY_ROOTFS}/tasks_ws  
    
#     # The building of these packages specificly is necessary due to their bad handling of dependencies.
#     # Described here: https://github.com/micro-ROS/micro-ROS-Agent/issues/161
#     # They both want to clone something during build, which is not possible due to no internet connection
#     colcon build --symlink-install --packages-select microxrcedds_agent --event-handler console_direct+ --cmake-args -DUAGENT_P2P_PROFILE=OFF -DUAGENT_CED_PROFILE=OFF -DUAGENT_USE_SYSTEM_FASTDDS:BOOL=ON -DUAGENT_USE_SYSTEM_FASTCDR:BOOL=ON -DUAGENT_BUILD_EXECUTABLE=OFF -DUAGENT_ISOLATED_INSTALL:BOOL=OFF
#     source install/local_setup.bash
#     colcon build --symlink-install --packages-up-to micro_ros_agent --event-handler console_direct+ --cmake-args -DMICROROSAGENT_SUPERBUILD=OFF
#     source install/local_setup.bash
    
#     # Normal build process
#     colcon build --symlink-install
# else
#     ros2 launch mir_navigation mir_nav_sim_launch.py world:=maze
# fi

# Copy the ws build in container to user folder
# cp -r /cirp_ws /$HOME

cd /$HOME/cirp_ws

colcon build

source /$HOME/cirp_ws/install/setup.bash

ros2 launch mir_gazebo mir_gazebo_launch.py world:=maze &
ros2 launch mir_navigation amcl.py use_sim_time:=true map:=$(ros2 pkg prefix mir_gazebo)/share/mir_gazebo/maps/maze.yaml &
ros2 launch mir_navigation navigation.py use_sim_time:=true
wait
exit