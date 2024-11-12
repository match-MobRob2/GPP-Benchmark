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

if [ -f $GPPP_CATKIN_WS_PATH/install/setup.bash ]
then
    source $GPPP_CATKIN_WS_PATH/install/setup.bash
fi

export ROS_DOMAIN_ID=9

if [[ $TASK == "copy" ]]
then
    cp -r -n /cirp_ws/src $GPPP_CATKIN_WS_PATH/src # Copy but dont overwrite the GPP-Pipeline repo

elif [[ $TASK == "rebuild" ]]
then
    echo "rebuild"
elif [[ $TASK == "start" ]]
then
    cd $GPPP_CATKIN_WS_PATH
    
    echo "Start Pipeline"
    colcon build
    source install/setup.bash

    # ros2 launch gpp_pipeline pipeline.launch.py
    python3 $GPPP_CATKIN_WS_PATH/src/GPP-Pipeline/container/start_pipeline.py
    # python3 $HOME/cirp_ws/src/GloPaPlan-Testing-Pipeline/pipeline/pipeline.py
    # ros2 launch mir_gazebo mir_gazebo_launch.py world:=maze
    # ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf"

    # ros2 launch mir_gazebo mir_gazebo_launch.py world:=maze &
    # ros2 launch mir_navigation amcl.py use_sim_time:=true map:=$(ros2 pkg prefix mir_gazebo)/share/mir_gazebo/maps/maze.yaml &
    # ros2 launch mir_navigation navigation.py use_sim_time:=true
    echo "THIS IS AFTER"
    # killall -9 gzserver
    # killall -9 gzclient
    kill $(ps aux | grep 'ign gazebo gui' | awk '{print $2}')
    # ros2 launch gpp_pipeline pipeline.launch.py
elif [[ $TASK == "positions" ]]
then
    cd $GPPP_CATKIN_WS_PATH
    
    echo "Create positions"
    colcon build
    source install/setup.bash

    ros2 launch gpp_pipeline create_positions.launch.py
    kill $(ps aux | grep 'ign gazebo gui' | awk '{print $2}')
elif [[ $TASK == "test" ]]
then
    cd $GPPP_CATKIN_WS_PATH
    
    echo "Start Pipeline"
    colcon build
    source install/setup.bash

    ros2 launch gpp_pipeline test_pipeline.launch.py
    kill $(ps aux | grep 'ign gazebo gui' | awk '{print $2}')
elif [[ $TASK == "data_analysis" ]]
then
    cd $GPPP_CATKIN_WS_PATH
    
    echo "Start Data Analysis"
    # colcon build
    source install/setup.bash

    python3 $GPPP_CATKIN_WS_PATH/src/GPP-Pipeline/data_analysis/read_rosbag_test.py
    # kill $(ps aux | grep 'ign gazebo gui' | awk '{print $2}')
elif [[ $TASK == "data_compare" ]]
then
    cd $GPPP_CATKIN_WS_PATH
    
    echo "Start Data Compare"
    # colcon build
    source install/setup.bash

    python3 $GPPP_CATKIN_WS_PATH/src/GPP-Pipeline/data_analysis/compare_rosbags.py
fi


# This setting is absolutly mandatory for the cluster!
# The cluster is not blocking or ROS messages from one user to the other
# If this is not set other users can see my topics and also the simulations mess each other up (time jump detected)
# This prevents it in a nices way than running "singularity run --net --network none match-dt-tasks.sif XX"
# export ROS_LOCALHOST_ONLY=0

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

# cd /$HOME/cirp_ws

# colcon build

# source /$HOME/cirp_ws/install/setup.bash



wait
exit