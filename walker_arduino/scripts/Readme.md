
# Check info from device connected as 'ttyUSB4'
```bash
udevadm info --attribute-walk --path=/sys/bus/usb-serial/devices/ttyUSB4
```
# Try rules written at '/etc/udev/rules.d' folder
```bash
udevadm control --reload-rules && udevadm trigger
```

# Virtual terminal to see output from new device
```bash
screen /dev/left_handle 115200
```
# Solve problem arduinos not appearing as ttyUSBx
Remove brtty
