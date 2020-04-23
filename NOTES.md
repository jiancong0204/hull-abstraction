# Notes Perfect ROS Repository

## Definitions

1. Dokuportal
    1. Tutorial
        1. Dokuportal
        1. Linux
        1. Setup your Account
        1. Qt Creator
        1. C++
            1. node handle basiertes threading
            1. c++ threading
            1. shared pointer/pointer etc.
        1. TemplatePackage
        1. ROS Essentials -> Link zum ROS Tutorial -> Danach Verweis auf unser Template Repository
            1. parameter
                1. config
                1. load from parameter server
                1. dynamic reconfigure
            1. launch
            1. Publisher
            1. Subscriber
            1. Service Client
            1. Service Server
            1. Action Server
            1. Action Client
            1. msg, srv, actions generation
            1. gtests
            1. capabilities and applications
        1. ROS Packages
            1. TF: Listener and Subscriber
            1. URDF, SDF
            1. Move It
            1. Move Base
            1. Localization (amcl)
            1. etc.
        1. Doxygen
        1. Summary
    1. Unit Documentation (e.g. UR5, iGPS, SLAM etc.)
        * IGM Specialist
        * Source Code Link
        * Doxygen Link
        * External Links
        * Technical Documentation: Logins, Components, Turn on, Turn off
1. GitLab Repository
    1. arduino [optional]
        * folder for arduino sketch files
    1. matlab [optional]
        * folder for matlab m files
    1. ros
        1. action
            1. CustomExample.action
        1. cfg
            1. SimpleTemplateNode.cfg
        1. config
            1. publisher_node.yaml
        1. include
            1. template_package
                1. libsimple_example_class
                1. simple_template_node
        1. launch
            1. publisher_node.launch
        1. msg
            1. CustomExample.msg
        1. src
            1. libsimple_example_class
            1. simple_template_node
                1. simple_template.cpp
            1. tools
            simple_template_node.cpp
        1. srv
            1. CustomExample.srv
        1. test
            1. libsimple_example
                1. conduct_heavy_computation_test.cpp
        1. CMakeLists.txt
            * C++ 11
        1. package.xml
            * Format 2
    1. .gitignore
            * html/
            * .html
            * ~
            * .log
            * .bin
    1. .gitlab-ci.yml
        * automatic doxygen creation for development branch
        * Generation of UML class diagrams
    1. doc
        1. Doxyfile
            * Correct the include path of header files that it can be used directly in ROS (add the package name es prefix) (idea by Henrik)
            * Add @todo and @bug list to the start

1. Doxygen:
    * The CI will automatically run for the **development** branch
    * The following lists are created for our documentation
        * todo
        * bug
        * test
1. ROS:
    1. Always use Package XML format 2
    1. Always use c++11
    1. Namespaces for securing wrong includation (Henrik) -> Result: ALWAYS use namespaces of package
    1. Include handling and folder structures
        -> package_name/include/package_name -> necessary and important for including headers of other packages
        -> "global" headers (inlclude) and "local" headers (src)
            -> put all libraries into include
            -> keep node header file in src
    1. Order of public, private and protected in header file
        -> public
        -> protected    
        -> private
    1. Cpp template classes in header or cpp (Sascha)
        Conclusion:
        -> Standard without template -> Header, Cpp
        -> Template Class -> Only header
        * Normal Class with template functions -> header (with template functions) + Cpp
    1. Rules for Cpp splitting for node class
        * Use only one cpp files and do everything in it (Small Nodes)
            * For the simple_template_node we have the following cpp:
                * simple_template.cpp

        * Split into multiple cpps (Big Nodes)
            * The following cpps are allowed in addition to the node cpp:
                * dynamic_reconfigure.cpp
                * publications.cpp
                * services.cpp
                * subscriptions.cpp
                * actions.cpp

---

## ToDo

1. Dokuportal: Read access for all users that have the link for the repository (still need to ask for getting access to the repo) (Florian)
    * Source Code linked in Doxygen automatically (Tobi)
    * Example: Doxygen of Perfect ROS Repository -> Source Code of [do_something_action_node](http://coar.pages.igm.rwth-aachen.de/perfect_ros_repository/do__something__action__node_8cpp_source.html)

1. Doxygen: 
    1. header files with correct include paths -> add package path before header name (Tobi)
        * Solved! Needs to be done for every repo itself
    1. Create complete class diagram (Tobi)
        * Solved! Each class should have its complete class diagram now
    1. Simplification of doxygen possible? -> Tests for best practice (Tobi)
        * Pending!
    1. How to make filter for msg/srv/action executable when cloning? (Henrik)
1. summit_bringup
    * global launch files Doxygen? -> Try to find solution (Tobi)
    * Pending!
1. Make template package compilable and runable (Henrik)
    * Done? Done! 
1. Put Threads into same structure as defined during last meeting (Johanna)
    * Done?
1. Find solution for action clients and action services within our template (Henrik)
    * Done? Done (80%)
1. Package XML Package format 2 (Marius)
    * Issues? No!
    * Whats new in version 2? "run_depend" and "build_depend" tags can now be replaced by a simple "depend" tag
    * Should we use format 2? Yes, the ROS tutorials are already using only format 2
    * In Indigoo catkin_create_pkg always creates format 1?
    * Follow the migration guide from format 1 to 2: http://docs.ros.org/jade/api/catkin/html/howto/format2/migrating_from_format_1.html#migrating-from-format1-to-format2
    * Henrik & Marius will do a final evaluation (most likely format 2)
1. ROSTESTS: still available or deprecated? -> Henrik
    * Deprecated? --> nice for testing overall nodes, or for example publishing rate
    * particularly selftest is promising
    * at the end it's a launch file
    * to be examined by ALL (particularly for integration purposes with multiple nodes)
1. CMakeLists: If clause for tests -> Henrik
    * Possible? --> For release build might save some time -> recommended by Henrik 
1. CMakeLists
    * What is the correct way to link the self-written library? (Henrik)
        * coar_libs repository
        * put to ROS_PACKAGE for easier integration
        
    * Want to get rid of the whole cpp-classes inside of the executable definition (Sascha)
        * for now, sufficient to write cpp-classes seperately
        * might change in future
        * simple way to implement, but requires cmake to run when new file is added (is this even a problem?)

        ~~~
        file(GLOB_RECURSE do_something_big_sources "src/do_something_big/*.cpp")
        add_executable(${PROJECT_NAME}_do_something_big_node
            src/do_something_big_node.cpp
            ${do_something_big_sources}
        )
        ~~~

---

## Next Discussion Points:

1. Metapackages as a container for all the releated packages of e.g a robot (Henrik)
    1. default metapackages for robots? mobile robots and manipulators?
    1. Naming of packages that are related to a special unit: Name of robot always as prefix (Tobi)
        * summit_description -> URDF
        * summit_navigation -> MoveBase
        * summit_teleoperation -> PS3 Controller
        * summit_bringup -> global launch files
        * ...
    1. Henrik: if necessary, useful (summit already done in this way)

1. Shall we make a rule for our repo that we do not use `ros` as a folder anymore? We can just create a meta-package on the first level with a good name -> The difference between `real` ROS repos and our ones is very very close with this strategy!
    1. only add other folders like `matlab` and `arduino` if they exist -> they are not an issue when they exist within a ROS workspace
    1. This would mean that we also can define a metapackage for the whole repository (CMakeLists.txt and package.xml files on first level)

    --> we keep the ros folder (majority's wish)

1. Used Version of C++
    1. We need to define a C++ version for our whole development group -> Otherwise we will have many different compiler versions and perhaps some features (in higher C++ versions) that are not considered in older versions
        1. Options: C++ 98, C++ 03, C++ 11, C++ 14, C++ 17
        1. Open questions: 
            * Who is using which version? 
            * Is it clever to always go for the latest? 

                C++ 11 definitely -> Standard
                C++14? (auto, lambda(?) expressions are useful features)
                C++17? (ISO certified at the end of 2017 -> wait?)


    1. Is it enough to use this compile option within the CMakeLists.txt file or is there more to have in mind:
        
        ~~~
        add_compile_options(-std=c++11)
        ~~~

        * c++14 works with gcc6 
        * keep in mind warnings for ROS compapility
        * QtCreator 5.9 requires gcc4.9

1. To reduce compile time, we should move #include tags from the .h files to the respective .cpp files (if possible)

    * accepted

1. Extend the gitlab tutorial section with the correct use of master, development, feature_xyz branchs?  

    * definitely (Henrik: Tobi)

1. Reopen discussion about template classes (Henrik und Sascha)
    * There are other habbits in the online community? (Henrik)
    * No, but: Aufteilen in zwei Files wäre möglich, beispielsweise durch eine .tpp datei

    ~~~
    // x.h file

    template <typename T>
    class X {
        void add(T a, T b);
    };

    #include "x.tpp"


    // x.tpp file

    tempate <typename T>
    void X<T>::add(T a, Tb) {}
    ~~~


    * which file ending to chose: hpp (->pcl) oder tpp

    * to discuss: problem: h should be hpp --> TBD

    * .tpp seems to be common


1. We need a small example for understanding the basics of shared pointer! Who is doing this? :)  --> Sascha

1. Outlook for the future (Tobi): All of our rules will be implemented in a Style Checking Test Suite that will run for every push onto the GitLab via CI. This means that all of our repos will converge towards this example repository slowly but surely!

1. Shall we integrate clang formatting to our repos? (Tobi)

1. Shall we get rid of the following doxygen commands as they are automatically created by GitLab
    * @author
    * @version
    * @date
    
    -> Only @author in the future

1. Shall define standard prefixes for the main ros data types?
    * Publisher: pub_*name*
    * Subscriber: sub_*name*
    * Service Client: client_*name*
    * Service Server: server_*name*
    * Action Client: aclient_*name*
    * Action Server: aserver_*name*
    * Messages: msg_*name*
    * Services: srv_*name*
    * Actions: action_*name*
    * Transformations: tf_*name*
    
    -> Contributing Guide and documentation in Tutorial but not strict rule 

1. Shall we use the "lib" prefix also for our folders within the /src folder? Or shall we only put it in front of the .so files?
    -> When you use "add_library" within the CMakeLists the prefix "lib" is created automatically

1. Is someone capable of creating a shell script or even GUI to ease the process of creating a package from the template (Automatically parsing placeholders, ...)

1. Merge all the folders into our big meta package [Henrik]

1. Fill content for description package [Tobi]

1. Member and global variables
    Idea Henrik:
        * Member: m_variable (ROS: variable_)
        * Global: g_variable

m_pub_example_publisher = nh.advertise...

m_energie = m_masse * m_gravitation * m_hoehe
energie_ = masse_ * graviation_ * hoehe_
energie = masse * graviation * hoehe

energie = masse_ * gravitation * hoehe_
energie = m_masse * graviation * m_hoehe

-> UNDERSCORE

- Ask Johanna if her version is done within the solution of Henrik of if anything is missing about threading?

---

## Notes during Meeting

1. 
     Alternative "Meeting" on Thursday, 14

     1. All Summits for testing --> More AP for WiFi
     1. Tobi and Marius prepare the novel Kinect Mounting until Monday

1. We do not need "initialize" and "shutdown" anymore
    -> The "run" method is enough for every node -> everything else is done within the constructor or deconstructor
    -> When we always use smart pointer we do not need to change the default deconstructor

    -> Perhaps we need some information about data management and memory management (Henrik, Sascha)
    -> Going to be a new Repository as it is not ROS related
    -> New year: Move discussion about perfect ROS repository to SKILL SHARING discussion
        -> Qt Debugger
        -> Cpp Data Management
        -> Modern c++ features:
                Smart pointers
                Move semantics
                Lambdas
                Templates
                Initializer lists
        -> Data oriented design (dod)
            Dod in general
            Overgeneralization
            Last minute decision making
            Efficient data structures	
        -> Compilers
            Compiler options (gcc, clang, …)
            Compiler flags	
        -> Tools
            Compiler explorer
            Perf
            Google benchmark library

1. Next level for development
    -> We can create bash scripts for creating template-based new elements as nodes
        -> GUI possible (Marius)

---

## Best Practices

1. When you use a service and you do not know how long it takes for computation -> put it into a thread (Johanna)
1. Always use single package for custom message generation! -> Integrate it in the corresponding metapackage

---
    
## Explanations for documentation

* Package.xml -> Necessary for rosdep install etc. for finding dependent packages
