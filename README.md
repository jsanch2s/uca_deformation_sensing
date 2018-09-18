# Deformation sensing and control
This repository contains ROS packages to estimate the shape and control a deformable object using a robot manipulator.
To model deformations, we created a ROS wrapper of the [Vega FEM](http://run.usc.edu/vega/) library.
This code was developed as part of my PhD thesis in Université Clermont Auvergne (UCA), France, under the supervision of Prof. [Youcef Mezouar](http://youcef-mezouar.wixsite.com/ymezouar).

## Getting Started
Follow these instructions to see how to install, use and test the contents of this repository.

## Requirements
The project has been developed and tested under Ubuntu 14.04 and Ubunt 16.04. It uses the following
dependencies:

* [ROS](http://wiki.ros.org/ROS/Installation) (Tested under Indigo and Kinetic)
* [numpy/scipy](https://www.scipy.org/install.html)

### Optional
To estimate the force based on the [BioTac](http://wiki.ros.org/BioTac) sensor readings
* [tflearn](http://tflearn.org/installation/ (tensorflow==1.1.0))
* [shadow_robot](http://shadow-robot.readthedocs.io/en/latest/generated/shadow_robot/INSTALL.html)

To run unit tests:
* [pytest](https://docs.pytest.org/en/latest/getting-started.html)


## Demo
**1. Launch the deformation sensing demo:**

```
roslaunch udom_deformation_sensing deformation_sensing_interactive_demo.launch mesh_filename:=bar_hard
```

**2. Use the GUI to control the force exerted to the object as well as to  start/stop/reset the deformation.**

After clicking the `start` button you should see a mesh object like the one shown below.

![Interactive demo](https://raw.githubusercontent.com/jsanch2s/uca_deformation_sensing/master/doc/deformation_sensing.png)


See also, the packages inside the metapackages described at the beginning for examples of
specific usage.



## Testing
Each package has a test directory with integration and unit tests.
The integration tests can be run using [rostest](http://wiki.ros.org/rostest) and the unit tests can
be run using `pytest`.

## Coding style tests
These tests can be run using [roslint](http://wiki.ros.org/roslint)

## Authors

* **Jose Sanchez** - *Initial work*

## License
This project is licensed under the GPLv3 License - see the [LICENSE.md](LICENSE.md) file
for details.

