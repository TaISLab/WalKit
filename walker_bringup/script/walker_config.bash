# This script assumes you just sourced .walker_cfg.bash 
source $ROS2_WORKSPACE/install/setup.bash & source $ROS2_WORKSPACE/install/setup.sh
# find ip on interface wlan0 
sed -i "130s/.*/    url : \x27ws:\/\/$ROS2_IP:9090\x27/" $ROS2_WORKSPACE/src/WalKit/walker_web_gui/gui/index.html 
