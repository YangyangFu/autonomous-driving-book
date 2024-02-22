# get current project path by using the file path
# use the directory of this file as the project path
export PROJECT_PATH=$(cd `dirname $BASH_SOURCE`; pwd)
echo ${PROJECT_PATH}

# set up paths
export PYTHONPATH=${PROJECT_PATH}:${PYTHONPATH}
