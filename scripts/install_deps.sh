#!/bin/bash
set -e
# This script installs all the dependencies needed for ig_graph_slam package
# assuming you have already installed inspector_gadget and its dependencies.

u="$USER"

echo_green()
{
	eval string1="$1"
	echo -e "\E[1;32m${string1}"
	tput sgr0
}

install_python_vtk()
{
	sudo apt-get install python-vtk
}

install_kindr()
{
	echo_green "Installing\ kindr..."

	if [ ! -d /home/$u/software ]; then
		mkdir /home/$u/software
	fi

	cd /home/$u/software

	if [ ! -d /home/$u/software/kindr ]; then
		git clone https://github.com/ANYbotics/kindr.git
		cd kindr
		mkdir build
		cd build
		cmake ..
		make -j8
		sudo make install
		sudo ldconfig # refresh shared library cache.
	else
		echo "kindr is already cloned, do you want to rebuild? (y/n)"
		read rebuild_kindr
		if [ $rebuild_kindr == "y" ]; then
			cd kindr
			cd build
			cmake ..
			make -j8
			sudo make install
			sudo ldconfig # refresh shared library cache.
		else
			echo "Not updating kindr..."
		fi

	fi
	echo_green "Done\ installing\ kindr."
}

install_gtsam()
{
	echo_green "Installing\ gtsam..."

	if [ ! -d /home/$u/software ]; then
		mkdir /home/$u/software
	fi

	cd /home/$u/software

	if [ ! -d /home/$u/software/gtsam ]; then
		git clone https://bitbucket.org/gtborg/gtsam.git
		cd gtsam
		mkdir build
		cd build
		cmake ..
		make -j8
		sudo make install
	else
		cd gtsam
		echo "gtsam is already cloned, do you want to pull any new commits? (y/n)"
		read pull_gtsam_changes
		if [ $pull_gtsam_changes == "y" ]; then
			git pull origin master
			cd build
			cmake ..
			make -j8
			sudo make install
		else
			echo "Not updating gtsam..."
			cd build
			cmake ..
			make -j8
			sudo make install
		fi
	fi
	echo_green "Done\ installing\ gtsam."
}

install_ceres()
{
	echo_green "Installing\ ceres\ deps..."

	if [ ! -d /home/$u/software ]; then
		mkdir /home/$u/software
	fi

	# google-glog + gflags
	sudo apt-get install libgoogle-glog-dev
	# BLAS & LAPACK
	sudo apt-get install libatlas-base-dev
	# SuiteSparse and CXSparse (optional)
	# - If you want to build Ceres as a *static* library (the default)
	#   you can use the SuiteSparse package in the main Ubuntu package
	#   repository:
	sudo apt-get install libsuitesparse-dev
	# - However, if you want to build Ceres as a *shared* library, you must
	#   add the following PPA:
	#sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
	#sudo apt-get update
	#sudo apt-get install libsuitesparse-dev

	echo_green "Done\ installing\ ceres\ deps."

	echo_green "Installing\ ceres..."

	cd /home/$u/software

	if [ ! -d /home/$u/software/ceres-solver ]; then
		git clone https://ceres-solver.googlesource.com/ceres-solver
		cd ceres-solver
		git checkout -q 1.13.0
    mkdir -p build
    cd build
		cmake ..
		make -j8
		make test
    sudo make install
    sudo ldconfig # refresh shared library cache.
	else
		echo "ceres is already cloned, do you want to rebuild? (y/n)"
		read rebuild_ceres
		if [ $rebuild_ceres == "y" ]; then
			cd ceres-solver
			cd build
			cmake ..
			make -j8
			TEST_ARGS="-E 'bundle_adjustment|covariance|rotation'" # Skip the slowest tests
			make test
	    sudo make install
	    sudo ldconfig # refresh shared library cache.
		else
			echo "Not updating ceres..."
		fi

	fi
	echo_green "Done\ installing\ ceres."
}

install_geographiclib()
{
	echo_green "Installing\ geographiclib..."

	if [ ! -d /home/$u/software ]; then
		mkdir /home/$u/software
	fi

    GEOGRAPHICLIB_VERSION="1.49"
    GEOGRAPHICLIB_URL="https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-$GEOGRAPHICLIB_VERSION.tar.gz"
    GEOGRAPHICLIB_DIR="GeographicLib-$GEOGRAPHICLIB_VERSION"

    if (ldconfig -p | grep -q libGeographic.so.17 ); then
        echo "GeographicLib version $GEOGRAPHICLIB_VERSION is already installed."
    else
        echo "Installing GeographicLib version $GEOGRAPHICLIB_VERSION ..."
        cd /home/$u/software
        wget "$GEOGRAPHICLIB_URL"
        tar -xf "GeographicLib-$GEOGRAPHICLIB_VERSION.tar.gz"
        rm -rf "GeographicLib-$GEOGRAPHICLIB_VERSION.tar.gz"

        cd "$GEOGRAPHICLIB_DIR"
        mkdir -p BUILD
        cd BUILD
        cmake ..
        make -j8
        make test
        sudo make install
    fi
		echo_green "Done\ installing\ geographiclib..."
}

install_opencv()
{
	echo_green "Installing\ OpenCV\ 3.3.0..."

	if [ ! -d /home/$u/software ]; then
		mkdir /home/$u/software
	fi

	cd /home/$u/software

	if [ ! -d /home/$u/software/opencv ]; then
		mkdir opencv
		cd opencv
		wget https://github.com/opencv/opencv/archive/3.3.0.zip
		unzip 3.3.0.zip
		rm -rf 3.3.0.zip
		cd opencv-3.3.0/
		mkdir build
		cd build
		cmake -D CMAKE_BUILD_TYPE=Release ..
		make -j8
		sudo make install
	else
		echo "opencv is already cloned, do you want to rebuild v3.3.0? (y/n)"
		read rebuild_opencv
		if [ $rebuild_opencv == "y" ]; then
			cd opencv
			if [ ! -d /home/$u/software/opencv/opencv-3.3.0 ]; then
				wget https://github.com/opencv/opencv/archive/3.3.0.zip
				unzip 3.3.0.zip
				rm -rf 3.3.0.zip
				cd opencv-3.3.0/
				mkdir build
				cd build
				cmake -D CMAKE_BUILD_TYPE=Release ..
				make -j8
				sudo make install
			else
				cd opencv-3.3.0/
				rm -rf build
				mkdir build
				cd build
				cmake -D CMAKE_BUILD_TYPE=Release ..
				make -j8
				sudo make install
			fi
		else
			echo "Not rebuilding opencv..."
		fi

	fi
	echo_green "Done\ installing\ opencv."
}

install_libwave()
{
	echo_green "Installing\ libwave..."

	if [ ! -d /home/$u/software ]; then
		mkdir /home/$u/software
	fi

	cd /home/$u/software

	if [ ! -d /home/$u/software/libwave ]; then
		echo "Cloning nickcharron's fork of libwave."
		git clone --recursive https://github.com/nickcharron/libwave.git
		cd libwave
		echo "do you want to fetch upstream?(y/n)"
		read fetch_upstream

		if [ $fetch_upstream == "y" ]; then
			git remote add upstream https://github.com/wavelab/libwave.git
			git fetch upstream
			git merge upstream/master
			git push origin master
		else
			echo "not fetching upstream."
		fi
		mkdir -p build

	else
		echo "libwave is already cloned."
		cd libwave

		echo "do you want pull any new commits?(y/n)"
		read pull_libwave_changes

		if [ $pull_libwave_changes == "y" ]; then
			git pull origin master
		else
			echo "not updating local repository."
		fi
		echo "WARNING: You may want to fetch upstream changes to this repo then rerun this script."
		echo "recompiling libwave..."
	fi

	cd build
	cmake ..
	make -j8
	echo_green "installing\ libwave."
	sudo make install
}

main()
{
	# install GTSAM and other required programs
	install_kindr
	install_python_vtk
	#install_gtsam
	install_ceres
	install_geographiclib
	install_opencv

	# install libwave
	install_libwave

	echo_green "DONE\ INSTALLING\ DEPENDENCIES\ FOR\ \IG_GRAPH_SLAM!"

}

main
