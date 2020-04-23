

# template_moveit_planner

This package contains a template Moveit Planner.


### How to use this template

#### Rename files and classes
To customize this package, use automatic renaming with regular expression. Choose a meaningful name for your package, e.g. "asdf_planner".

1. Rename the filenames and folder names. Replace "*template_moveit*" with your planner name, e.g "*asdf*" (Do NOT include "*planner*" here).
```
	find . -iname "*template_moveit*" -exec rename 's/template_moveit/asdf/' '{}' \;
```
	There will be an error like 'folder template_moveit not found'. **Repeat the command several times** until nothing gets displayed in the console. Thats because some of the found file paths were changed during the execution and are no longer valid.

2. Replace all occurences of "*template_moveit*" with "*asdf*" in all files  ( this may also change this file, so make a copy first)
```
	find . -type f -exec sed -i 's/template_moveit/asdf/g' {} +
```

3. Replace all occurences of "*TEMPLATE_MOVEIT*" with "*ASDF*" in all files  ( this may also change this file, so make a copy first)
```
	find . -type f -exec sed -i 's/TEMPLATE_MOVEIT/ASDF/g' {} +
```

4. Replace all occurences of "*Template_moveit*" with "*Asdf*" in all files  ( this may also change this file, so make a copy first)
```
	find . -type f -exec sed -i 's/Template_moveit/Asdf/g' {} +
```


#### Integrate your planner into Moveit
* Copy the `.launch.xml` file from the *files_to_copy_elsewhere*-folder into the `your_robot/your_robot_moveit_config/launch` folder.
* Copy the `.yaml` file from the *files_to_copy_elsewhere*-folder into the `your_robot/your_robot_moveit_config/config` folder.
* Delete the files_to_copy_elsewhere folder
* Edit the file `your_robot/your_robot_moveit_config/launch/move_group.launch` and rename the value of the pipeline arguement:
```
<include ns="move_group" file="$(find lexium_sts_moveit_config)/launch/planning_pipeline.launch.xml">
	<arg name="pipeline" value="asdf" />
</include>
```
* Copy this package into your ROS-workspace and build it.

* You should be able to use run moveit now. There is a simple interpolating planner already provided within this package, so you can start planning now.

#### Write your own Planner
* Have a look at the simple example class `TEMPLATE_MOVEITPlannerInterp`. You can add your planner like this one to the  C++ only library `template_moveit_library`. Make sure you inherit your planner class from `TEMPLATE_MOVEITPlannerBase`.
