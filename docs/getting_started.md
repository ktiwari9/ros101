Getting started with ROS {#GettingStartedSection} 
===============

After you have read through this page, you should be able to do these things __on your own__:
1. Get ROS up and running on your system (any version of Ubuntu)
2. Setup your first ROSject & compile it successfully

__Prerequisites:__
  - Ability to use Linux and the terminal
  - Familiarity with the ROS [jargon](@ref JargonSection)

# Installing ROS and setting up your system

Now that the jargon/technical terminology of the ROS framework have been introduced, it is time to get your hands dirty. To do this, you would first need to install ROS on a PC which has:
- `Ubuntu >= 14.04 LTS`
- `Python == 2.7` **ROS does not support Python 3. For Python 3 you must use ROS2 which is not included in this crash course.**
- `Suitable Graphics drivers for running ROS simulators`
- Proper installation (sudo) permissions to install ROS packages
- Free storage space (around 1.5 GB)

You can find detailed instructions on how to install ROS on your system on the official [ROS wiki](http://wiki.ros.org/). However, you can also follow the few steps below to get the installation process done quickly, easily and hopefully hassle free. Notice that there are many versions of ROS, called *distros*. Each ROS distro is specialized for a particular version of Ubuntu.

## Installation
1. To get started as quickly as possible with this tutorial, clone the repo if you haven't done so yet:

        git clone https://gitlab.com/ktiwari9/ros101.git

2. The installation is done by following the official [ROS installation](http://wiki.ros.org/ROS/Installation) will require some amount of work and time. Hence, we provide a script with the repository that does the installation automatically. Run the script __twice__ to get ROS, its dependencies and [MoveIt!](https://moveit.ros.org/) installed. (The second script execution installs the MoveIt!). It is __not__ recommended to run the script with sudo, because doing it might lead to permission errors later on.

        ./autoInstallROS.sh

3. Create a catkin workspace in desired location by following the steps in *Initializing a new catkin workspace* below.

4. __OPTIONAL:__ Using ROS will usually require a multitude of terminal windows, which can become quite messy with the conventional Linux terminal. To make it a little more organized, it might make sense to use the `terminator` terminal, which allows for multiple terminal windows in one neat window. Install it by: 

        sudo apt-get install terminator

## Initializing a new catkin workspace

First we create a directory for the new workspace if one does not exist yet. (Quite commonly the workspace lies in home directory and is called `catkin_ws` or `catkin_workspace`, but the workspace can have any name you want. Here the \<CATKIN WORKSPACE> refers to the name of your unique workspace).


    mkdir -p <CATKIN WORKSPACE>/src 

move into src/

    cd <CATKIN WORKSPACE>/src

ROS command for auto-generating all the required files and folders for the new workspace
    
    catkin_init_workspace 

Move to the root of the catkin workspace

    cd .. 

Compile all files in your catkin workspace (you must be in the workspace root directory)

    catkin_make

In order for ROS to recognize your catkin workspace, you need to run

    source <CATKIN WORKSPACE>/devel/setup.bash

If you did the steps correctly, then you should have a functional catkin workspace (farmland) with the three subfolders: `build/`, `devel/` and `src/` like in the directory structure below:

    catkin_ws/
    ├── build/
    ├── devel/
    │   ├── setup.bash
    │   ├── setup.sh
    │   └── setup.zsh
    └── src/

You need to execute one of the setup.bash, setup.sh and setup.zsh __in each terminal window__ that runs something from the catkin workspace in question. This can be quite a hassle to do, so it is recommended to automate it by inserting the source command from above to your `~/.bashrc`. 


# Starting new ROS projects aka ROSjects

ROS is language independant which allows use of several different languages such as *C++ and Python*. It is possible to use mix of languages at the same time if wanted. However, in this tutorial, we will only focus on Python, because it is easier to get started with and it does not require us to compile the ROSject each time we want to run it. ROS also supports MATLAB.

All ROSjects reside in the catkin workspace. A catkin workspace with several ROSjects can look something similar to this directory structure:

    catkin_ws/
    ├── build/                  # All build files end up here after catkin_make or catkin build
    │   ├──...
    │   ....
    ├── devel/
    │   ├── env.sh
    │   ├── lib
    │   ├── setup.bash          # Activate this catkin workspace with one of these (depending on the shell you are using)
    │   ├── setup.sh            # Activate this catkin workspace with one of these (depending on the shell you are using)
    │   ├── _setup_util.py
    │   └── setup.zsh           # Activate this catkin workspace with one of these (depending on the shell you are using)
    └── src/                    # Folder for all your ROSjects
        ├── CMakeLists.txt      # Build instructions for the whole catkin workspace (symlink). Don't touch this one.
        ├── rosject_1/          # Here resides one ROSject
        │   ├── CMakeLists.txt  # Build instructions for rosject_1. Feel free to edit this one (as long as you know what you are doing)
        │   .....
        ├── rosject_2/
        │   ├── CMakeLists.txt
        │   .....
        └── rosject_3/
            ├── CMakeLists.txt
            .....


## Creating a new ROSject

__Note:__ On the next page, you are going to do a practical exercise regarding to the following contents. Hence, you can relax and just read through this page. Don't hestiate to come back when doing the excercise.

To create a new ROSject do the following:

    cd <CATKIN WORKSPACE>/src
    catkin_create_pkg ​ <package_name>​ dependency1 dependency2 ... dependencyn

The dependencies are other ROSjects, that are required for the project. Usually these are at least rospy (for Python) and roscpp (for C++).
The resulting directory structure for a ROSject using Python is the following:

    └── <ROS PACKAGE NAME>
        ├── CMakeLists.txt          # This contains all the settings for building the ROSject (even when Python is used). Feel free to edit this file whenever necessary.
        ├── include                 # This contains all C++ header files
        │   └── <ROS PACKAGE NAME>  
        │       └── functions.h     
        ├── __init__.py
        ├── package.xml             # General information of the ROSject. Feel free to edit this if you need to make changes to ROSject settings.
        ├── setup.py                # This is executed by catkin_make, if the line 'catkin_python_setup()' in CMakeLists.txt is uncommented
        ├── src                     # This is the folder for all the nodes.
        │   ├── node1.py
        │   ├── node2.py
        │   ├── node3.cpp           # Nodes can be in Python or C++, however you want.
        │   └── node4.cpp
        └── scripts                 # This is the directory for Python scripts, i.e. Python programs which are not nodes. Can also be imported into any other ROS package
            └── script1.py          # Python script, with non-ROS related Python functions

## Configuring your new ROSject for Python

Most of this structure above is autogenerated by `catkin_create_pkg` command, but some files you need to create yourself. This includes generally:
1. Create the `__init__.py` files (can be completely empty) according to above directory structure
    - This forces Python to recognize all `.py` files in that directory as Python packages.
2. Create the setup.py file and paste the following code:

        #!/usr/bin/env python
        from distutils.core import setup
        from catkin_pkg.python_setup import generate_distutils_setup
        setup_args = generate_distutils_setup(
            packages=['<ROS PACKAGE NAME>'],
            package_dir={'': 'src'}
        )
        setup(**setup_args)

    __Note:__ remember to change \<ROS PACKAGE NAME> to the name of your ROSject.


3. On all Python nodes and scripts, make them executable. For example with:

        chmod +x file.py
        
    This step ensures that the catkin knows how to build and execute the ROSject

4. In the project CMakeLists.txt uncomment the catkin_python_setup()

5. Update the CMakeLists.txt to compile your nodes and consider them as executables
  For Python nodes:

        install(PROGRAMS
        src/node1.py
        src/node2.py
        scripts/script1.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

6. There might be several other things to edit in the CMakeLists.txt file, depending on the complexity of your ROSject and its dependencies, libraries etc. Remember that catkin_make reads the CMakeLists.txt file and compiles the ROSject accordingly. Do not expect catkin_make to know anything other than what is specified in this file. The file contains a lot of useful instructions as comments, which will help you in most common cases. Making more advanced changes to CMakeLists.txt than previously mentioned might require knowledge of how cmake works.

# Recommended next step

"This sounds quite straightforward on a conceptual level", you might think, "...but how do I actually apply this in practice? How do I plant my apple tree, exactly how I want it?" 

Well, move on to the next chapter and you will find out [here](@ref ExampleSection) .