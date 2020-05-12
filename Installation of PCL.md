# How to install PCL in Ubuntu 16.04
## 1. Install required dependencies

```
sudo apt-get install git build-essential linux-libc-dev
sudo apt-get install cmake cmake-gui
sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
sudo apt-get install mpi-default-dev openmpi-bin openmpi-common
sudo apt-get install libflann1.8 libflann-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libboost-all-dev
sudo apt-get install libqhull* libgtest-dev
sudo apt-get install freeglut3-dev pkg-config
sudo apt-get install libxmu-dev libxi-dev
sudo apt-get install mono-complete
sudo apt-get install libglew-dev
sudo apt-get install libsuitesparse-dev
```

## 2. Complie and install VTK 6.2.0

> "apt-get install" in ubuntu 16.04 can only install the version of 5.10. However, version of 6.2.0 or above is required for pcl.
* 2.1. Prepare the source code of VTK

```
wget https://www.vtk.org/files/release/6.2/VTK-6.2.0.tar.gz
tar -xvzf VTK-6.2.0.tar.gz
sudo mv VTK-6.2.0 /opt
cd /opt/VTK-6.2.0
```

* 2.2. Compile VTK
```
sudo mkdir build
cd build
sudo cmake ..
sudo make
sudo make install
```

> "make -j(#Cores of your CPU)" can be used to speed up compilation process, e.g. make -j6. However, you must make sure that there is enough memory for all the parallel jobs. 

## 3. Download the source code of PCL-1.8.0

## 4. Compile the source code

```
cd pcl
sudo mkdir build
cd build
sudo cmake-gui
sudo make
sudo make install
``` 

> Make sure that "BUILD_surface_on_nurbs" and "USE_UMFPACK" are both selected in cmake configuration

