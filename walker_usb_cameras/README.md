# USB CAMERA CONFIGURATION PACKAGE

The aim of this package is to contain all calibration and configuration files needed to use an usb camera into walker system. 
This file is mostly a summary from info taken at:

[usb cam github](https://github.com/ros-drivers/usb_cam/tree/ros2)

[navigation tutorials: calibrating the camera](https://navigation.ros.org/tutorials/docs/camera_calibration.html)

TODO: stereo cams. There is a tutorial [here](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) that can be summarised.
## Check USB Camera
First step is check that camera works on our system.

### First terminal. 
Launch camera node without configuration:
```console
ros2 run usb_cam usb_cam_node_exe --ros-args -r __ns:=/camera
```


That command will show all possible formats, resolutions and fps and will start publishing data on ROS2.
```console
[INFO] [1688469843.164735462] [usb_cam]: Starting 'default_cam' (/dev/video0) at 640x480 via mmap (yuyv) at 30 FPS
[INFO] [1688469843.340893099] [usb_cam]: This devices supproted formats:
[INFO] [1688469843.340996505] [usb_cam]: 	YUYV 4:2:2: 640 x 480 (30 Hz)
[...]
[INFO] [1688469843.341301354] [usb_cam]: 	YUYV 4:2:2: 800 x 600 (20 Hz)
[...]
[INFO] [1688469843.341391412] [usb_cam]: 	YUYV 4:2:2: 1280 x 960 (5 Hz)
[...]
[INFO] [1688469843.341704075] [usb_cam]: 	Motion-JPEG: 800 x 600 (30 Hz)
[...]
[INFO] [1688469843.341899575] [usb_cam]: 	Motion-JPEG: 1280 x 960 (30 Hz)
[...]
[INFO] [1688469843.341927310] [usb_cam]: Setting 'brightness' to 50
[INFO] [1688469843.369696697] [usb_cam]: Setting 'white_balance_temperature_auto' to 1
[INFO] [1688469843.369725452] [usb_cam]: Setting 'exposure_auto' to 3
[INFO] [1688469843.378898988] [usb_cam]: Setting 'focus_auto' to 0
unknown control 'focus_auto'

[INFO] [1688469843.393069446] [usb_cam]: Timer triggering every 33 ms

```

Take note of the available formats, sizes and fps, as we will need them later on our "Params.yaml" file. 
Do not stop this command and open a second terminal.

### Second terminal. 
Here we will check that images are properly transmitted into ROS. First, lets check there are images:
```console
ros2 topic hz /camera/image_raw
```

We will see that FPS are similar to what was available.
```console
average rate: 29.813
	min: 0.032s max: 0.036s std dev: 0.00194s window: 31
average rate: 29.788
	min: 0.032s max: 0.036s std dev: 0.00195s window: 61
average rate: 29.819
	min: 0.032s max: 0.036s std dev: 0.00194s window: 91
```

Second command, see if the topic actually contains images:
```console
ros2 topic bw /camera/image_raw
```

We should see a decent amount of MB/s going around:
```console
Subscribed to [/camera/image_raw]
18.92 MB/s from 29 messages
	Message size mean: 0.61 MB min: 0.61 MB max: 0.61 MB
18.35 MB/s from 58 messages
	Message size mean: 0.61 MB min: 0.61 MB max: 0.61 MB
18.38 MB/s from 88 messages
	Message size mean: 0.61 MB min: 0.61 MB max: 0.61 MB
```

Finally, check that the topic can be seen in a ros2 app:
```console
ros2 run rqt_image_view rqt_image_view
```

Camera seems to work. Now we are ready to build both configuration and calibration files.

## configuration file (params.yaml)

This is the configuration file used by ROS2. It defines the camera configuration used. For example for our camera at video0 a params.yaml file:

````json
/**:
    ros__parameters:
      video_device: "/dev/video0"                                                   # <----------------- EDIT THIS TO MACH YOUR SYSTEM
      framerate: 20.0                                                               # <----------------- Taken from usb_cam_node_exe command
      io_method: "mmap"
      frame_id: "camera"                                                            # <----------------- careful with multiple cam systems
      pixel_format: "yuyv"                                                          # see usb_cam package, supported_formats for list of supported formats
      image_width: 800                                                              # <----------------- Taken from usb_cam_node_exe command
      image_height: 600                                                             # <----------------- Taken from usb_cam_node_exe command
      camera_info_url: "package://walker_usb_cameras/config/jpc_camera_info.yaml"   # <----------------- Calibration file. See next section
      camera_name: "jp_camera"                                                      # <----------------- This name must match whatever you put in the info_url file
      brightness: -1
      contrast: -1
      saturation: -1
      sharpness: -1
      gain: -1
      auto_white_balance: true
      white_balance: 4000
      autoexposure: true
      exposure: 100
      autofocus: false
      focus: -1
````

camera_info_url file will be created by callibration in next section.

## calibration file (camera_info.yaml) 

First we need a checkerboard for calibration. You can check [calib.io website](https://calib.io/pages/camera-calibration-pattern-generator) to generate any pattern. Specifically, we are using a “8x10” checkboard with 20mm squares. 

Actually we use a 7x9 checkerboard with 20mm squares, but calibration uses the interior vertex points, so a “8x10” board uses the interior vertex parameter “7x9”.

In a well-lit area, clear of obstructions and other check board patterns, point the checkboard to the camera and launch the camera calibration node:

```console
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.02 --ros-args -r image:=/camera/image_raw -p camera:=/my_camera
```

Now move, tilt and rotate the checkboard around until the calibrate button is highlighted. 

Click it to see the results. It takes around the minute for calibration to take place.

After the calibration is completed the save and commit buttons light up. And you can also see the result in terminal.

Press the save button to see the result. Data is saved to “/tmp/calibrationdata.tar.gz”

Inside the tar file, “ost.yaml” file is our camera_info.yaml, save from the "camera_name" field that needs to match the value in "params.yaml" file.


## launcher file
Last step is to create a launch file that uses our params.yaml to configure our camera. There is one at launch folder that can be used as template, just by changing the params.yaml file.