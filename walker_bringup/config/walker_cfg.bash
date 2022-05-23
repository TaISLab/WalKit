# ~/.walker_cfg.bash
# you should set these accordingly
ROS2_WORKSPACE="~/workspace/walker_ws"
NETWORK_INTERFACE="wlan0"

#this should be automatic
ROS2_IP=$(/sbin/ip -o -4 addr list $NETWORK_INTERFACE | grep dynamic | awk '{print $4}' | cut -d/ -f1)