# ROS-101


This project gives a hands-on tutorial on ROS. The objective of this project is to gradually introduce the vast framework of ROS to an absolute beginner using visual tools and worked programming exercises in Python.

# Project Website
The [project website](https://ktiwari9.gitlab.io/ros101) spans across a primer to ROS and hands-on programming exercises along with a code API.

# Pre-requisites
- Ubuntu 16.04 LTS + ROS Kinetic
- Readup Doxygen Manual with doxypypy plugin
- `Python >=2.7 && <=3.0`

# Key Learnings
- ROS and its simulators
- Interfacing with robots
- Code documentation and API generation
- Version Control using Git

# Instruction for including packages
- `rosdoc-lite` will be used to generate doxygen documents for each package. For python package, doxypypy will be used as filter to generate documentation from python docstring. In order to enable this, the only extract requirement for each package is a rosdoc.yaml file in the package directory.
- the `rosdoc.yaml` should look like the one in the `sample_navigation` package.

# File Description
|   File/Folder Name  |                                      Description                                     |
|:-------------------:|:------------------------------------------------------------------------------------:|
|        docs/        |                Folder containing the documents for website generation                |
|    toy_rosject/     |           catkin_pkg hosting the ROS-Py nodes for toy ROSject using Youbot           |
|     navigation/     |     catkin_pkg housing a ROS-Py node for a simple point-to-point navigation task     |
| `autoInstallROS.sh` |             Shell script for fresh install for ROS for absolute beginners            |
|    `py_filter.sh`   | Shell script to parse the source files via doxypypy plugin to generate documentation |
|      `LICENSE`      |                License file for public release and code-reuse/sharing                |
|  `rosdoc_script.sh` |               Generating project lists with apt hyperlinks for website               |
|  `CONTRIBUTING.md`  |                         List of contributors for this project                        |