# Param: TASK | Mandatory
# This param defines the file name of the .sif container

if [ -z $1 ]
then
    echo "run.sh - Parameter 1 - task name - is missing"
    exit
else
    TASK=$1
fi

# Param: Container Name | Optional
# This param defines the file name of the .sif container

if [ -z $2 ]
then
    CONTAINER_NAME="cirp.sif"
else
    CONTAINER_NAME=$2
fi

apptainer run ../container/$CONTAINER_NAME $TASK