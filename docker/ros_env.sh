# Se carga en cada terminal del contenedor

source /opt/ros/humble/setup.bash

[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

export GAZEBO_MODEL_DATABASE_URI=""
