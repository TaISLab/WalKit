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
    - walker_plot: Publishes Arrow marker based on handle force readings.  
    - walker_py_cluster: (TODO)
    - walker_simulation: roller simulation in gazebo.
    - walker_slam: (TODO)
    - walker_step_detector: reports step position.
    - walker_web_gui: web interface for tests.
            
- Repositories forked for this project
    - aws-robomaker-hospital-world: simulated hospital for gazebo.  
    - bwt901cl_pkg: IMU package.   
    - csm: ROS2 port of Andrea Censeis Scan matcher, needed by laser scan matcher package.
    - leap_motion: stereo camera package (DOES NOT WORK WITH arm64)  
    - ros2_laser_scan_matcher: provides odometry from laser readings.
    - rplidar_ros2: RPLidar A1 package. 

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
