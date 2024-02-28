# How to call this script:
# source source_bridge.sh

# This script is used when you want to run or compile the ros1_bridge.

# Adjust this value if your roscore is located elsewhere
ROS_MASTER_URI=http://localhost:11311

echo "Note: Any warnings about not mixing ROS environments can be ignored."
echo "This is because we actually want to source both in order to bridge them."
echo "Sourcing ROS1 (Noetic)"
source /opt/ros/noetic/setup.bash
echo "Sourcing ROS2 (Foxy)"
source /opt/ros/foxy/setup.bash
echo "Exporting ROS_MASTER_URI to ${ROS_MASTER_URI}"
export ROS_MASTER_URI=${ROS_MASTER_URI}

echo "Sourcing local ROS1 workspace"
source source_ros1.sh

echo "Sourcing local ROS2 workspace"
source source_ros2.sh

echo "Sourcing bridge_ws"
SETUPPATHBRIDGE="${SCRIPTPATH}/ros1_bridge_ws/install/setup.bash"
if [ -f ${SETUPPATHBRIDGE} ]; then
  source ${SETUPPATHBRIDGE}
else
  echo "Could not source bridge_ws. This is ok if you didn't build it yet. Just run this script again afterwards."
  cd ./ros1_bridge_ws/
  colcon build --symlink-install --cmake-force-configure
  source ./install/setup.bash
  cd ..
fi

