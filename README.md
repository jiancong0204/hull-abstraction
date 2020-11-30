# Efficient Abstraction of 3D Point Clouds by Hull Generation

![COAR Logo](doc/assets/coarlogo.png "Coar Logo")

---

## Content

* [Introduction](#introduction)
* [Installation](#installation)
* [Hull Generation Methods](#hull-generation-methods)
* [Benchmark](#benchmark)

---

## Introduction

This repository is part of the Center of Advanced Robotics by the IGMR - Aachen University. The aim is to implement several hull generation methods and compare their performances. 

In this repository, following hull generation methods are implemented,

- Greedy triangulation algorithm
- B-spline surface fitting
- Poisson reconstructio
- Marching cubes algorithm

---

## Installation

A summary of installing necessary libraries for this package.
- VTK 6.2.0
- PCl 1.8.0

---

### 1. Install required dependencies

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
---

### 2. Complie and install VTK 6.2.0

> "apt-get install" in ubuntu 16.04 can only install the version of 5.10. However, version of 6.2.0 or above is required for pcl.

#### 2.1. Prepare the source code of VTK

```
wget https://www.vtk.org/files/release/6.2/VTK-6.2.0.tar.gz
tar -xvzf VTK-6.2.0.tar.gz
sudo mv VTK-6.2.0 /opt
cd /opt/VTK-6.2.0
```

#### 2.2. Compile VTK

```
sudo mkdir build
cd build
sudo cmake ..
sudo make
sudo make install
```

> "make -j(#Cores of your CPU)" can be used to speed up compilation process, e.g. make -j6. However, you must make sure that there is enough memory for all the parallel jobs.
 
---

### 3. Download the source code of PCL-1.8.0

---

### 4. Compile the source code

```
cd pcl
sudo mkdir build
cd build
sudo cmake-gui
sudo make
sudo make install
``` 

> Make sure that "BUILD_surface_on_nurbs" and "USE_UMFPACK" are both selected in cmake configuration

---

## Hull Generation Methods

A summary of the implemented hull generation methods.

---

### Greedy Triangulation Algorithm

- Based on local 2D projection.
- Assumption of locally smooth surface.
- Assumption of relatively smooth transitions between areas with different point densities.

 #### Steps:
 
1. Given a point p, its normal vector and the tangent plane perpendicular to the normal vector are determined.
2. Point p as well as its vicinity is projected to the tangent plane.
3. Edges are formed between each pair of points and then arranged in order of length.
4. The shortest edge is going to be removed from the memory. If it does not intersect any of the current triangulation edges, it will be added to the triangulation before being removed.
5. Repeat step 4 until the memory is empty.
6. Two-dimensional connection relationship is now established. It can then be converted into three-dimensional space.

---

### B-spline Surface Fitting

- Given a set of point P.
- P is projected to a plane to obtain another set of point PP.

#### Steps:

1. Find values for the control points, denoted by cp, that minimize the distance between PP and B-spline curve c(cp). This curve will be used as contour in the trimming step later.
2. Find values for the control points (cp) that minimize the distance between P and B-spline surface s(cp).
3. The result of surface fitting is a four-sided shape which is larger than the desired shape. It can be removed by trimming away areas that lie outside the contour.

---

### Poisson Surface Reconstruction

- Reconstruction by solving a spatial Poisson system.
- Highly strong to data noise.
- Given a point cloud with normal estimation.

#### Basic Ideas:

- Compute a 3D indicator function which is defined as 1 at points inside the model and 0 at points outside.
- The gradient of the indicator function is a vector filed that is zero almost everywhere except for the boundary, where the gradients are equal to the normals of boundary.
- The problem is now reduced to finding the function whose gradient best approximates a known vector field (normals of point cloud).
- Further reduction to a Poisson problem: finding the function whose divergence of gradient equals divergence of the given vector field.

---

### Marching Cubes

- Extract iso-surface from the indicator function

#### Paradigm:

1. The space is divided into small cubes. Each cube has eight vertices.
2. Based on how the boundary intersects the cube, each vertex of each cube is assigned a binary value (e.g. 0 if the point is thought to lie outside the surface.).
3. The situation of intersection here is determined using point cloud instead of the surface.

---

## Final Report

See [here](https://github.com/jiancong0204/masters-thesis)

