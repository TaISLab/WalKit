
udevadm info --attribute-walk --path=/sys/bus/usb-serial/devices/ttyUSB4
udevadm control --reload-rules && udevadm trigger
screen /dev/left_handle 115200