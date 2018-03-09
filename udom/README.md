# UCA Deformable Object Manipulation (UDOM)
This repository contains ROS packages to control the shape of a deformable object through
robotic manipulation. It was developed as part of my PhD thesis in Université Clermont Auvergne (UCA), France, under the supervision of Prof. Youcef Mezouar.

The packages of this project are divided into the following metapackages:

1. **udom_perception:** Packages that process the output of sensors (e.g. tactile sensors) to interpret a physical phenomenon or estimate a quantity such as the contact force.

2. **udom_modeling:** Packages used to compute the deformation of an object based on its deformation model and the output of a perception package/component (e.g. the contact force estimated by tactile sensors).

3. **udom_control:** Packages that handle the motion control of the robot in order to deform the object into a desired shape.

4. **udom_utils:** Packages required by the other metapackages to perform operations such as geometric computations (e.g. force transformation).


## License
This project is licensed under the GPLv3 License - see the [LICENSE.md](LICENSE.md) file
for details.

