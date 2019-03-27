#! /bin/bash


#================================================================
#   Copyright (C) 2018 * Ltd. All rights reserved.
#
#   File name   : autoInstallROS.sh
#   Author      : Kshitij Tiwari
#   Email       : kshitij.tiwari@aalto.fi
#   Created date: 2018-12-18 15:18:35
#   Description : Shell script to install ROS-distro-desktop full along with Moveit!
#
#================================================================
#set -x
#set -n 

function checkInstallation()
{
	if [ -z "$ROS_DISTRO" ]; then 
		# Common ROS env var not set: start ROS installation
		selectRosVer
	else
		echo "-------------------------------------------------"
		echo "  ROS-$ROS_DISTRO has been installed already."
		echo "  Checking for MoveIt!..."
		echo "-------------------------------------------------"

		# Install MoveIt!
		sudo apt-get install -y ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-commander ros-$ROS_DISTRO-moveit-kinematics ros-$ROS_DISTRO-moveit-planners ros-$ROS_DISTRO-moveit-visual-tools
		sudo apt-get install -y ros-$ROS_DISTRO-moveit-ros ros-$ROS_DISTRO-moveit-plugins ros-$ROS_DISTRO-moveit-setup-assistant ros-$ROS_DISTRO-moveit-runtime
		while [ $? -ne 0 ]
		do
			echo "Install moveit failed!!, trying again... /n"
			sleep 10
			sudo apt-get install -y ros-$ROS_DISTRO-moveit-commander ros-$ROS_DISTRO-moveit-experimental ros-$ROS_DISTRO-moveit-kinematics ros-$ROS_DISTRO-moveit-planners
			sudo apt-get install -y ros-$ROS_DISTRO-moveit-ros ros-$ROS_DISTRO-moveit-plugins ros-$ROS_DISTRO-moveit-setup-assistant ros-$ROS_DISTRO-moveit-runtime
		done

		sleep 6
	fi
}

function selectRosVer()
{
	ROSdistro=""
	UbuntuVer=$(lsb_release -rs)
	if [[ "$(lsb_release -is)" == "Ubuntu" ]]; then
		echo "Detected operating system: $(lsb_release -is) $UbuntuVer"
		read -r -p "Is this correct? [Y/n] " userSelection
		if [[ "$userSelection" =~ ^([yY][eE][sS]|[yY]+$) || "$userSelection" == "" ]]; then
			if (( ${UbuntuVer%%.*} >= 18 )); then
					ROSdistro="melodic"
			elif (( ${UbuntuVer%%.*} >= 16 )); then
					ROSdistro="kinetic"
			elif (( ${UbuntuVer%%.*} >= 14 )); then
					ROSdistro="jade"
			elif (( ${UbuntuVer%%.*} >= 12 )); then
					ROSdistro="indigo"
			else
				echo "Matching version cannot be resolved automatically"
			fi
		fi
	fi

	# If automatic ROS distro selection failed, try manual
	if [[ -z "$ROSdistro" ]]; then
		echo "Switching to manual select"
		echo "[1] melodic: Ubuntu 18.04 and 18.10"
		echo "[2] kinetic: Ubuntu 16.04 and 16.10"
		echo "[3] jade:    Ubuntu 14.04 and 14.10"
		echo "[4] indigo:  Ubuntu 12.04 and 12.10"
		read -r -p "Select suitable ROS version [1-4] " userSelection
		if [[ "$userSelection" =~ ^([0-9]+$) ]]; then
			case "$userSelection" in
			1)
				ROSdistro="melodic";;
			2)
				ROSdistro="kinetic";;
			3)
				ROSdistro="jade";;
			4)
				ROSdistro="indigo";;
			*)
				echo "Invalid selection."
				return 1;;
			esac
		else
			echo "Invalid selection. (Select with numbers)."
			return 1
		fi
	fi

	installRos
}

function installRos()
{
	echo "Starting automatic installation, do not close the terminal..."
	sleep 3
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
	while [ $? -ne 0 ]
	do
		echo "add key failure!!, try again... /n"
		sleep 10
		sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
	done
	sudo apt update
	sudo apt-get install -y ros-$ROSdistro-desktop-full
	while [ $? -ne 0 ]
	do
		echo -e "Install desktop-full failure, try again.../n"
		sleep 10
		sudo apt-get install -y ros-$ROSdistro-desktop-full
	done
	sudo rosdep init
	rosdep update
	while [ $? -ne 0 ]
	do
		echo -e "rosdep update failure, try again... /n"
		sleep 10
		rosdep update
	done
	if [ $USER = kshitij ]; then
		echo ".bashrc has been configure already!"
	else
		echo "#config ros system env" >> ~/.bashrc
		echo "source /opt/ros/$ROSdistro/setup.bash" >> ~/.bashrc
	fi
	source ~/.bashrc

	sudo apt-get install -y  python-rosinstall python-rosinstall-generator python-wstool build-essential
	echo -e "Installed ROS ($ROSdistro) sucessfully! \n"
	return 0
}

checkInstallation

