# hull_abstraction

![COAR Logo](doc/figures/COAR_Doxygen.png "Coar Logo")

This repository is part of the Center of Advanced Robotics by the IGMR - Aachen University. The main purpose is to define folder structures, coding rules and documentation habbits as a template package.

---

# 1. Installation

A summary of install instructions necessary for the template package


## 1.1 Init Submodule
The [IGMR Robotics System Toolbox](https://igm-git.igm.rwth-aachen.de/COAR/igmr_packages/igmr_robotics_system_toolbox) is required to compile the ros template nodes. If you haven't already cloned and build the toolbox, you can init and update the submodule:
~~~py
cd perfect_ros_repository/ros/modules/igmr_robotics_system_toolbox/
git submodule init
git submodule update
~~~

---

~~##  1.1 Gtest~~

~~NOTES:~~
~~* Simply using `catkin_add_gtest` in a CMakeLists.txt file (no need for find_package(GTest)) works with GTest out of the box. ~~
~~* The `libgtest-dev` package is already installed by ROS.~~
~~* Also note that installing pre-compiled GTest libraries is discouraged [by Google](https://github.com/google/googletest/blob/master/googletest/docs/FAQ.md#why-is-it-not-recommended-to-install-a-pre-compiled-copy-of-google-test-for-example-into-usrlocal).~~


~~Gtest is required for building the test cases. Install Gtest as decribed [here](http://ysonggit.github.io/coding/2014/12/19/use-gtest-in-ros-program.html)~~


~~cd /usr/src/gtest~~
~~sudo apt-get install libgtest-dev~~
~~sudo cmake CMakeLists.txt~~
~~sudo make~~
~~#copy or symlink libgtest.a and ligtest_main.a to /usr/lib folder~~
~~sudo cp *.a /usr/lib~~

---

## 1.2 Google Benchmark

Google Benchmark is required for building and running the benchmarks. Install Google Benchmark as described [here](https://github.com/google/benchmark)

In the desired install folder (usually /opt), run the following commands, or paste this into a shell script:

~~~py
git clone https://github.com/google/benchmark.git
cd benchmark
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make
sudo make install
~~~

## 1.3 Set up GCC version alternatives

To be able to use the latest improvements in the most commonly used C and C++ compiler GCC/G++, such as C++ 14/17 features, install the newest reslease version (e.g. currently 7.2):

~~~py
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-7 g++-7
~~~

__Do not delete older versions, unless you know what you are doing!__ If you have older versions installed and you want to keep them, set up all possible versions using update-alternatives. This will create symbolic links to your installations, so that you can easily switch between them.
To see which versions of GCC/G++ are installed on your system, use

~~~py
ls /usr/bin/ | grep ^gcc\-[0-9]
~~~

To set up the symbolic links, first remove all previously created links using

~~~py
sudo update-alternatives --remove-all gcc
sudo update-alternatives --remove-all g++
~~~

Then, create new links for all desired versions. You also have to specify a priority for each version. Choose priorities between 0 and 100 depending on your preferences, such that in auto mode,
Ubuntu will choose whatever version you desire. As an example, we will assume we installed GCC/G++ 4.9 and 7 and want to set 7.x as the default version. In this case we can use the following commandos:

~~~py
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 60 --slave /usr/bin/gcc gcc /usr/bin/gcc-7
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.9 40 --slave /usr/bin/gcc gcc /usr/bin/gcc-4.9
~~~

By using the "master/slave" setup, we can make sure that whenever a G++ version is chosen, the corresponding GCC version is chosen as well.
If you want to keep G++ and GCC separate, install both update alternatives independently (recommended only for advanced users).
The priority is specified following the (master) install, here we chose a priority of 60 for version 7.x and 40 for version 4.9.

We can now easily switch between versions using

~~~py
sudo update-alternatives --config g++
~~~

and choosing one of the displayed alternatives.

---

# 2. Demo

## 2.1 Run minimal_publisher_node

__Launch:__

~~~
roslaunch minimal_publisher_pkg minimal_publisher.launch
~~~

---

# 3. Tests

__Build:__

~~~
catkin_make tests
~~~

__Launch:__

To start all available tests, use

~~~
roslaunch template_nodes run_unit_tests.launch
~~~

__Run:__

To start a specific test, use the common syntax used to start a rosnode (using one of the template tests):

~~~
rosrun template_nodes conduct_heavy_computation_test
~~~

---

# 4. Benchmarks


## 4.1 General use

The basic use-pattern for benchmarks is:

~~~
rosrun template_nodes benchmarks <arguments>
~~~

---

## 4.2 Running a subset of benchmarks

Per default, all benchmarks specified in the benchmarks.cpp will be run. To run a subset of benchmarks, use:

~~~
rosrun template_nodes benchmarks --benchmark_filter= <regex>
~~~

where __\<regex\>__ defines a regular expression

### 4.2.1 Regex examples

Useful examples using regular expressions:

Regular Expression | Explanation
------------ | -------------
__\^\<name\>__ | Matches benchmarks starting with __\<name\>__
__\<name\>$__ | Matches benchmarks ending with __\<name\>__
__\^\<name\>$__ | Matches benchmark __\<name\>__ exactly
__\<name\>[0-9]{1,*n*}__ | Matches benchmarks containing __\<name\>__ followed by a number with a length of __*n*__ or less (e.g. 0-999 for __*n*__ = 3)

---

### 4.2.2 Other useful arguments

A list of commonly used flags to pass to a benchmarks file:

Benchmark Argument | Explanation
------------ | -------------
__--benchmark_repetitions=*n*__ | The specified benchmarks are run *n* times. Reports the individual times, mean, median and standard deviation of the runs
__--benchmark_report_aggregates_only=*\[true/false\]*__ | For use with the argument above. When set to true, only mean, median and standard deviation are reported
__--benchmark_out_format=*\[console/json/csv\]*__ | Sets the output format. Per default format is set to console.
__--benchmark_out=<i>filename</i>__ | Save the output in the specified file. Does not supress console output.

---

## 4.3 CPU scaling warning

If you see the following warning, when running benchmarks:

~~~
***WARNING*** CPU scaling is enabled, the benchmark real time measurements may be noisy and will incur extra overhead.
~~~

your machine scales down your CPU-frequency to reduce energy-consumption. To disable scaling, use:

~~~
sudo cpupower frequency-set --governor performance
~~~

The scaling mode is reset on each startup, so you will do no permanent damage

You might need to install linux-tools-common:

~~~
sudo apt-get install linux-tools-common
~~~

---
