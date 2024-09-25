# Param: Container Name | Optional
# This param defines the file name of the .sif container

if [ -z $1 ]
then
    CONTAINER_NAME="cirp.sif"
else
    CONTAINER_NAME=$1
fi

# Param: Definition File Name | Optional
# This param defines the file name of the definition file
if [ -z $1 ]
then
    DEF_FILE_NAME="testing_pipeline.def"
else
    DEF_FILE_NAME=$1
fi

apptainer build ../container/$CONTAINER_NAME ../container/$DEF_FILE_NAME 