# This file should be modified and copied to ~/.walker_cfg.bash
# It is needed by walker_config.bash script, called by TMuLE (at package walker_bringup) during config. 
# These two variables should be adjusted accordingly. WARNING: Dont use '~' for home folder
ROS2_WORKSPACE="/home/ubuntu/workspace/walker_ws"
NETWORK_INTERFACE="wlan0"

# This should grab IP address on wireless interface. It is needed for the web interface
ROS2_IP=$(/sbin/ip -o -4 addr list $NETWORK_INTERFACE | grep dynamic | awk '{print $4}' | cut -d/ -f1)
