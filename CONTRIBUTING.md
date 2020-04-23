# Contribution Guide

The contribution guide defines the style conventions in this repository.

---

This Contributing Guide is the **central** Contributing Guide of the entire COAR Project at IGMR. Please only edit the root Contributing Guide under [the Perfect ROS Repository](https://igm-git.igm.rwth-aachen.de/COAR/best_practices/perfect_ros_repository/blob/master/CONTRIBUTING.md) for updates.

---

## Content

* [Build Rules](#build-rules)
* [Repository Rules](#repository-rules)
* [C++ Rules](#c++-rules)
* [Formatting](#formatting)
* [Header Guards](#header-guards)
* [ROS Rules](#ros-rules)
* [Naming Rules](#naming-rules)
* [Package Rules](#package-rules)
* [Node Rules](#node-rules)
* [Console Output](#console-output)
* [Documentation Rules](#documentation-rules)
* [Files](#files)
* [Classes](#classes)
* [Structs](#structs)
* [Functions](#functions)
* [Variables](#variables)
* [To Dos](#to-dos)
* [Bugs](#bugs)
* [Node Files](#node-files)
* [Msg Files](#msg-files)
* [Srv Files](#srv-files)
* [Action Files](#action-files)

---

## Build Rules

1. The whole source code has to successfully built against the IGMR ROS Docker Container Image

---

## Repository Rules

1. Every ROS related repository must contain the following files and folders:
    - doc/
        - figures/
        - Doxyfile
        - DoxygenLayout.xml
    - .gitignore
    - .gitlab-ci
    - CONTRIBUTING.md
    - LICENSE
    - qi-config.json
    - README.md
1. The README.md of a ROS related repository sticks to the following layout:
    - Installation
    - Demo
    - Tests
    - Benchmarks (optional)
1. Only the following directory names are allowed on the top-level of a ROS related repository:
    - arduino
    - doc
    - examples
    - matlab
    - ros
    - tools
    - [additional directories are possible but need to be discussed within the COAR group]
1. The repository sticks to the following branch conventions:

| **Branch name** | **Function** |
|---------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| master | The master branch is the release branch. It will store versions for milestones, reports and closed tasks. It is **NOT** the branch for our daily work |
| development | The development branch is used for our daily work and the ongoing development towards the next milestone, report or closed task. It is the default branch for every of our repositories! |
| feature_[name_of_feature] | This branch is branched off the development branch. It is used for the implementation of a new feature e.g. “feature_environment_parser” within the CPCS repo. This workflow is essential because everyone’s work is unaffected by the work of others |
| bug_[name_of_bug] | This branch is branched off the development branch. It is used for the fix of bugs that has been encountered during the ongoing process. This workflow is essential because everyone´s work is unaffected by the work of others |
| thesis_[name_of_student] | This branch is branched off the development branch. It is used for a complete thesis of a student who is developing some new features within a thesis |


---

## C++ Rules

### Formatting

1. Indent each block by 4 spaces
2. Never insert literal tab characters
3. Braces, both opening and closing, go on their own lines. **No cuddled braces**

```javascript
if (a < b)
{
    //do stuff
}
else
{
    //do other stuff
}
```

4. Braces can be omitted if the enclosed block is a single-line statement, e.g.:

```javascript
if (a < b)
    x = 2*a;
```

5. Always include the braces if the enclosed block is more complex, e.g.:

```javascript
if (a < b)
{
    for (int i = 0; i < 10; i++)
        ROS_INFO_STREAM("The value of i is " << i);
}
```

---

**clang-format**

A `.clang-format` is contained in this repository, and it's recommended to use
`clang-format` for formatting your source code automatically. This means you
don't have to adjust your indent and newline by hand anymore.

There are a few options for invoking clang-format:

* Manually: Run `clang-format --style=.clang-format -i my_source_file.cpp`
* In your IDE:
    * vim: for example, [vim-clang-format](https://github.com/rhysd/vim-clang-format)
    * QtCreator: [Beautifier](https://doc.qt.io/qtcreator/creator-beautifier.html)
    * Visual Studio Code: [Clang-Format](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format)
    * Visual Studio: [built-in](https://devblogs.microsoft.com/cppblog/clangformat-support-in-visual-studio-2017-15-7-preview-1/)

---

### Header Guards

1. All C++ headers must be protected against multiple inclusion by #ifndef guards, e.g.

```javascript
#ifndef PACKAGE_NAME_FILE_NAME_H
#define PACKAGE_NAME_FILE_NAME_H

// Your Code

#endif // PACKAGE_NAME_FILE_NAME_H

```

This guard should begin immediately after the license statement, before any code is written. It should end at the end of the file!

---

## ROS Rules

### Naming Rules

1. Every ROS related element sticks to the following naming conventions

| **Object** | **Naming** | **Example** |
|---------------------------------|-------------------------|------------------------------------|
| ROS Packages | under_scored | my_package_example |
| ROS Topics/Services/Actions | under_scored | example_topic |
| ROS Messages, Services, Actions | CamelCased | PointCloud.msg |
| ROS Nodes | under_scored | move_base |
| Files | under_scored | example_file.cpp |
| Libraries | under_scored Exception | libmy_great_thing |
| Classes | CamelCased | ExampleClass |
| Classes with Acronym | CamelCased Exception | HokuyoURGLaser |
| Function | camelCased | int exampleMethod(int example_arg) |
| Function arguments | under_scored | int exampleMethod(int example_arg) |
| Variables | under_scored | bool my_example_variable |
| Constants | ALL_CAPITALS | NETWORKPORT |
| Member variables | under_scored trailing _ | int example_int_ |
| Global variables | under_scored leading g_ | int g_shutdown |
| Namespaces | under_scored | robotic_arm_ns |

---

### Package Rules

1. For a generic ROS package that contains at least one node the following directories are allowed:
    - cfg
    - config
    - include
    - launch
    - src
    - test
    - benchmark
1. General allowed suffixes for ROS package names are:
    - _actions
    - _bringup
    - _common
    - _control
    - _description
    - _environment
    - _gazebo
    - _msgs
    - _navigation
    - _nodes
    - _simulation
    - _srvs
    - _ui
1. For msg packages the following rules are defined:
    1. The pkg name will end with a `_msgs` suffix
    1. The package only contains the following subfolder:
        - `msg/`
1. For srv packages the following rules are defined:
    1. The pkg name will end with a `_srvs` suffix
    1. The package only contains the following subfolder:
        - `srv/`
1. For an action packages the following rules are defined:
    1. The pkg name will end with a `_actions` suffix
    1. The package only contains the following subfolder
        - `action/`
1. For a robot_description packages the following rules are defined:
    1. The pkg name will end with a `_description` suffix
    1. The package only contatins the following subfolders:
        - meshes
        - urdf
        - robots
        - launch
1. The robot meshes are split into two groups:
    - dae files for the visual information
    - stl files for the collision information

---

### Node Rules

1. ROS Nodes are seperated into a node file and into a node class (.cpp and .h)
1. The node class files are stored within a subdirectory of the package's /src named after the node file
1. Node classes are not available for other packages (the header is not stored in the /include path of the package)
1. Node files always ends with `_node.cpp`
1. Node files always instanciate an object of the node class
1. Node files always run the `run` method of the node class object
1. Node classes always have a `run` method that executes the node functionality

---

### Console Output

1. Always use `rosconsole` commands instead of printf and friends. The reasons for this are listed here:
    - color-coded
    - controlled by verbosity level and configuration file
    - published on /rosout, and thus viewable by anyone on the network (only when working with roscpp)
    - optionally logged to disk

---

## Documentation Rules

### Files

Every C++ header file is documented with the following information

```cpp
/**
 * @brief Example documentation of a file
 *
 * Here is plenty of space for any detailed desciption of this file! Here you can describe the functionalities of this file and the intention why you did certain things this way
 *
 **/
```

### Classes

Every class is documented with the following information within the C++ header file

```cpp
/**
 * @brief Example class documentation
 *
 * Detailed class description
 **/

class ExampleClass
{
...
};
```

### Structs

Every struct is documented with the following information within the C++ header file

```cpp
/**
 * @brief Example struct documentation
 *
 * Detailed struct description
 *
 **/

struct my_struct
{
    bool        test; //!< This is a bool test variable
    int         flag; //!< This is an int flag
    std::string foo;  //!< This is a foo string
}
```

### Functions

Every function is documented with the following information within the C++ header file

```cpp
/**
 * @brief Adding two integers
 * @param one First integer [1]
 * @param two Second integer [1]
 * @return Sum of One and Two [1]
 **/
int add_two_ints(int one, int two)
{
   int result = one + two;

   return result;
}
```

### Variables

Every variable is documented with the following information within the C++ header file

```cpp
double example_variable; //!< Example variable for the demonstration of the variable documentation
```

### To Dos

To Dos are used for signing open tasks inside of the code

```cpp
/** @todo Check if this variable is still necessary */
int todo_var; //!< Explanation of todo variable
```

### Bugs

Bugs are used for signing bugs inside of the code

```cpp
/** @bug This variable should be double. Cast it within the whole file to make it possible */
int bug_var; //!< Explanation of bug variable
```

### Node Files

Every C++ file that contains a ROS Node implementation is documented with the following information

```cpp
/**
 * @file
 * @brief <add brief description here>
 * @ros_node <Name of ROS node>
 *
 * Detailed Description of the Nodes logic and purpose
 *
 **/
```

### Msg Files

Every ROS Msg file is documented with the following information

```
## @file
## @ros_msg
##
## <Details>

## @name Fields
##@{

Header header ##< ROS Msg Header
# Add your msg variables here

##@}

```

### Srv Files

Every ROS Srvs file is documented with the following information

```
## @file
## @ros_srv
##
## <Details>

## @name Request
##@{

int32 req ##< Define request variables here

##@}
---
## @name Response
##@{

int32 resp ##< Define response variables here

##@}
```

### Action Files

Every ROS Action file is documented with the following information

```
## @file
## @ros_action
##
## <Details>

## @name Goal
##@{

int32 goal ##< Define the goal variables here

##@}
---
## @name Result
##@{

int32 res ##< Define the result variables here

##@}
---
## @name Feedback
##@{

int32 feedback ##< Define the feedback variables here

##@}
```

---
