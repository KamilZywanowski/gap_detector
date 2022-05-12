xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="auke/epfl2"
docker build -f Dockerfile -t $ROS_IMAGE .
