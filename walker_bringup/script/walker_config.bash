# This script assumes you just sourced .walker_cfg.bash 
source $ROS2_WORKSPACE/install/setup.bash & source $ROS2_WORKSPACE/install/setup.sh
# CoW: This writes wireless ip address on the web interface. 
# Now we use a fixed wireless ip
# TODO: make web interface to find out its own ip address.
# sed -i "240s/.*/    url : \x27ws:\/\/$ROS2_IP:9090\x27/" $ROS2_WORKSPACE/src/WalKit/walker_web_gui/gui/index.html
