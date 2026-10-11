# udev rules for the Walkit serial sensors

The launch files and tmule sessions expect fixed device names, independent of the
`/dev/ttyUSBx` number the kernel assigns on each plug-in:

| Device      | Name             | Used by                                                   |
|-------------|------------------|-----------------------------------------------------------|
| RPLidar A1  | `/dev/rplidar`   | `rplidar_ros` (`serial_port:=/dev/rplidar`, see `tmule/*.yaml`) |
| WitMotion   | `/dev/witmotion` | `witmotion_ros` (`port: witmotion`, `config/wt901*.yaml`) |

There is no ready-made rules file in this package: the USB IDs depend on the exact
adapter of each unit, so each machine creates its own rule following the steps below.

## 1. Find the USB IDs of the device

Plug **one** device at a time and look at what appears:

```bash
lsusb                              # new line: "ID vendor:product  description"
ls /dev/ttyUSB* /dev/ttyACM*       # its device node, e.g. /dev/ttyUSB0
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial'
```

Take `idVendor` and `idProduct` from the first block (the USB device itself, not its
parents). If `lsusb` shows the same `vendor:product` for both devices (e.g. two CP2102
adapters, `10c4:ea60`), they can only be told apart by `ATTRS{serial}`, or, if the serial
is identical too, by the physical USB port (`KERNELS=="1-2.3"`, from the `udevadm info -a`
output). In that case always plug each device in the same port.

## 2. Write the rules

Create `/etc/udev/rules.d/99-walker.rules`, replacing `XXXX`/`YYYY` with the IDs found:

```
# RPLidar -> /dev/rplidar
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", SYMLINK+="rplidar", GROUP="dialout", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1"

# WitMotion IMU -> /dev/witmotion
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", SYMLINK+="witmotion", GROUP="dialout", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1"
```

- `SYMLINK+=` gives the device its `/dev/<name>`.
- `GROUP="dialout"`, `MODE="0660"` make it accessible to every user of the `dialout` group.
- `ENV{ID_MM_DEVICE_IGNORE}="1"` keeps ModemManager from probing the port.
- To disambiguate, add `ATTRS{serial}=="..."` (or `KERNELS=="..."`) to the rule.

## 3. Apply and check

```bash
sudo usermod -aG dialout $USER      # once; log out and in again
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/rplidar /dev/witmotion   # symlinks to ttyUSBx, group dialout
```

## Troubleshooting

- Test a rule without unplugging: `udevadm test $(udevadm info -q path -n /dev/ttyUSB0)`.
- CH340 adapters (`1a86:7523`) can be grabbed by `brltty` on Ubuntu, which makes the port
  disappear right after plugging. If `dmesg` shows "brltty": `sudo apt remove brltty`.
- Permission denied on the port: `id` must show `dialout` (a new login is needed after
  `usermod`).
