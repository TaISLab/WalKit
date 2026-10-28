# WalKit
[![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/)

<p align="center">
  <img src="https://github.com/TaISLab/WalKit/blob/master/Pictures/walkerTransfiereReduced.jpg" />
</p>

WalKit is an open hardware project created to monitor the elderly. It is modular, so different parameters, such as weight-bearing, cadence, activity duration, walking speed or other spatiotemporal gait parameters can be monitoring. 

- Packages in WalKit repository:
    - firmware: arduino codes for handles and wheels. 
    - hardware: 3d models and schematics.
    - walker_arduino: ros2 nodes to interact with USB arduino boards.   
    - walker_bringup: robot specific configuration scripts and tools. 
    - walker_centroid_support: obtains weight centroid based on current support.
    - walker_description: robot description.  
    - walker_diff_odom: odometry node.
    - walker_loads: reports partial supports on each leg.
    - walker_msgs: custom handle and wheel encoder msgs.
    - walker_navigation2: nav2 configuration for walker (TODO). 
    - walker_rviz: draws what the walker measures in rviz (handle loads, feet, support centroid, stability, gait) and holds the rviz views.
    - walker_simulation: roller simulation in gazebo.
    - walker_slam: (TODO)
    - walker_step_detector: reports step position.
    - walker_web_gui: web interface for tests.
            
- Other repositories the workspace uses (all in [`walker.repos`](walker.repos), see [Getting the sources](#getting-the-sources))
    - csm: ROS2 port of Andrea Censi's scan matcher, needed by the laser scan matcher package.
    - ros2_laser_scan_matcher: provides odometry from laser readings.
    - rplidar_ros: RPLidar A1 package.
    - witmotion_ros: driver of the WitMotion IMU.
    - slg_msgs, laser_segmentation: scan segmentation, used by the `seg` variant of walker_step_detector.
    - mocap4r2: MOCAP4ROS2 messages, visualisation and Vicon driver.
    - e4_msgs: messages of the Empatica E4 wristband.

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- GETTING THE SOURCES -->
## Getting the sources

The public repositories the workspace is made of (WalKit itself, the sensor drivers, the laser odometry, the mocap messages...) are
listed in [`walker.repos`](walker.repos), in the native format of [vcstool](https://github.com/dirk-thomas/vcstool). Each key is the
path inside the workspace's `src` folder, and `version` is a branch, a tag or a commit:

```yaml
repositories:
  lidar/rplidar_ros:
    type: git
    url: https://github.com/Slamtec/rplidar_ros.git
    version: ros2
```

To get all of them into an empty workspace (WalKit included, so there is nothing to clone before):
```
sudo apt install python3-vcstool
mkdir -p ~/workspace/walker_ws/src && cd ~/workspace/walker_ws
vcs import src --input https://raw.githubusercontent.com/TaISLab/WalKit/jazzy/walker.repos
```
From a clone of WalKit it is `vcs import src < src/WalKit/walker.repos`. The destination folder (`src`) has to exist already. Then build the
workspace with colcon as usual.

| To... | Run |
|---|---|
| see what changed in every repository | `vcs status src` |
| update them all | `vcs pull src` |
| add the repositories that were added to the file, without touching the ones you have | `vcs import --skip-existing src < walker.repos` |
| download without history | `vcs import --shallow src < walker.repos` |
| check that every url and version of the file exists | `vcs validate --input walker.repos` |
| save the exact commits of your workspace (e.g. for an experiment) | `vcs export --exact src > workspace.repos` |

Things to know:
- **Do not use `--skip-existing` for the first import.** `witmotion_ros` brings an empty `witmotion-uart-qt` folder (its submodule), the file
  has an entry for that path, and `--skip-existing` skips folders that exist: it stays empty and `witmotion_ros` does not build. Use it to
  update, because it also leaves alone the repositories where you have checked out another branch; a plain `vcs import` over a workspace
  switches each repository back to its `version`.
- `witmotion/witmotion_ros/witmotion-uart-qt` is pinned to a commit (the one `witmotion_ros` has as a submodule), which `vcs validate`
  cannot verify without cloning. `--shallow` works with it.
- `vcs` prints a `pkg_resources` deprecation warning with recent setuptools: it is harmless.
- Commented-out entries are the repositories the workspace used before or does not use any more; removing the `#` at the start of their
  lines makes them valid entries.
- The mocap4w analysis tools are in private repositories and are not in this file; they are listed in `mocap4w.repos`, in the
  [mocap_analysis](https://github.com/MFernandezCarmona/mocap_analysis) repository (it needs access to that account). Import it after this one.
- The old `walker.rosinstall` (wstool format) is gone: `wstool` is deprecated and does not read `.repos` files.

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage
The system can be launched with the following:
```
cd ~/walker_ws/src/WalKit/walker_bringup/config/
tmule -c walker.yaml launch --tag core
```
If you want to automatically start recording a rosbag as well, remove 'tag' option. They will be stored at '~/bagFolder'.

ROS launch files can be monitored at the tmux session. You can attach to it with the following command:
```
tmux a -t walker_ros2
```

You may have noticed we use Marc-Hanheide's [TMuLE](https://github.com/marc-hanheide/TMuLE) program to configure and manage a [tmux](https://github.com/tmux/tmux) session running all the components. TMuLE can be installed with:
```
sudo -H pip3 install tmule
```

<p align="right">(<a href="#top">back to top</a>)</p>


<!-- CONTRIBUTING -->
## Contributing

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- LICENSE -->
## License

This project is licensed under the Creative Commons Attribution 4.0 - see [LICENSE](https://github.com/TaISLab/WalKit/blob/master/LICENSE) file for details.

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Manuel Fernandez-Carmona - [@ermanazas](https://twitter.com/ermanazas) - manolofc at gmail.com

Project Link: [https://github.com/TaISLab/WalKit/tree/humble](https://github.com/TaISLab/WalKit/tree/humble)

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

This work is funded by [Programa Proyectos RETOS del Ministerio de Ciencia, Innovación y Universidades (SAVIA: Sistema de Autonomía Variable para movIlidad Asistida, Ref: RTI2018-096701-B-C21)](http://www.grupoisis.uma.es/index.php?option=com_jresearch&view=project&task=show&id=208&Itemid=18&lang=es) at Málaga University, Spain

[Plan Propio de la Universidad de Málaga (E3-PROYECTOS DE PRUEBA DE CONCEPTO (E3/02/18))](https://www.uma.es/servicio-de-investigacion/cms/menu/plan-propio-de-investigacion/?set_language=en) at Málaga University, Spain

This great readme is based on [Othneil Drew](https://github.com/othneildrew/Best-README-Template)'s README template.

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TaISLab/WalKit
[contributors-url]: https://github.com/TaISLab/WalKit/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TaISLab/WalKit
[forks-url]: https://github.com/TaISLab/WalKit/network/members
[stars-shield]: https://img.shields.io/github/stars/TaISLab/WalKit
[stars-url]: https://github.com/TaISLab/WalKit/stargazers
[issues-shield]: https://img.shields.io/github/issues/TaISLab/WalKit
[issues-url]: https://github.com/TaISLab/WalKit/issues
[license-shield]: https://img.shields.io/github/license/TaISLab/WalKit
[license-url]: https://github.com/TaISLab/WalKit/blob/master/LICENSE
[product-screenshot]: images/screenshot.png
