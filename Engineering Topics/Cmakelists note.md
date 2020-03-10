>[ROS catkin CMakeLists syntax](../Uncategorized/catkin_CMakeLists.txt%20-%20ROS%20Wiki.pdf)  
>[catkin document](http://docs.ros.org/kinetic/api/catkin/html/index.html)


<mark>Capitalized names specify the argument options </mark>
## Overall Structure
Your CMakeLists.txt file **MUST** follow this format otherwise your packages will not build correctly. The order in the configuration **DOES** count. 
1. Required CMake Version (cmake_minimum_required()) 
2. Package Name (project()) 
3. Find other CMake/Catkin packages needed for build (find_package()) 
4. Enable Python module support (catkin_python_setup()) 
5. Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
6. Invoke message/service/action generation (generate_messages())

## Project Name
Project name is specified by project(). One can refer to this variable by ${PROJECT_NAME}

## Finding Dependent CMake Packages
[find_package()](https://cmake.org/cmake/help/latest/command/find_package.html) is used to find dependent CMake packages for building the package.
As for catkin based packages in ROS, one must include on required CMake package "catkin":

```
find_package(catkin REQUIRED [COMPONENTS])
```
REQUIRED is an option provided by CMake to specify the compiler to stop if the corresponding package is not found.  
A package-specific list of required components may be listed after the COMPONENTS option (or after the REQUIRED option if present). 

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)
```

The find_package() would automatically define related CMake environment variables if a package is found. These envrionemnt variables could be utilized later in the document.   
Several important variables are as follows:
- \<NAME\>_FOUND - Set to true if the library is found, otherwise false
- \<NAME\>_INCLUDE_DIRS or \<NAME\>_INCLUDES - The include paths exported by the package 
- \<NAME\>_LIBRARIES or \<NAME\>_LIBS - The libraries exported by the package 
- \<NAME\>_DEFINITIONS - ?

To include packages as components of catkin, the include paths, libraries, etc exported by the package are also appended to the catkin_ variables. For example, **catkin_INCLUDE_DIRS** contains the include paths not only for catkin but also for all other packages in the list.

### Boost Library
If using C++ and Boost, you need to invoke find_package() on Boost and specify which aspects of Boost you are using as components. For example, if you wanted to use Boost threads, you would say:
```
find_package(Boost REQUIRED COMPONENTS thread)
```
This will generate environment variables with Boost_ prefix such as Boost_INCLUDE_DIRS


## catkin_package()
[catkin_package()](https://docs.ros.org/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package) is a catkin-provided **CMake macro**. It installs the package.xml file, and it generates code for find_package and pkg-config so that **other packages can get information about this package**.

This function **must be called before** declaring any targets with add_library() or add_executable(). The function has 5 optional arguments:
- INCLUDE_DIRS(list of strings) - CMAKE_CURRENT_SOURCE_DIR-relative paths to C/C++ includes
- LIBRARIES(list of strings) - names of library targets that will appear in the catkin_LIBRARIES and ${PROJECT_NAME}_LIBRARIES of other projects that search for you via find_package.
- CATKIN_DEPENDS(list of strings) - Other catkin projects that this project depends on 
- DEPENDS(list of strings) - Non-catkin CMake projects that this project depends on. For a better understanding, see this explanation (http://answers.ros.org/question/58498/what-is-the-purpose-of-catkin_depends/). 
- CFG_EXTRAS - Additional configuration options

Example:
```
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES projlib1 projlib2
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen
  CFG_EXTRAS proj-extras[.cmake|.cmake.in|.cmake(.develspace|.installspace)?.em]
)
```

## Specify Build Targets
Build targets can take many forms, but usually they are two types: 
- Executable Target - programs we can run 
- Library Target - libraries that can be used by executable targets at build and/or runtime

### Custom Output Direcotry
```
set_target_properties(python_module_library PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYT HON_DESTINATION})
```

### Include Paths
```
include_directories(
    <dir1> 
    <dir2>
    ...
)
```
example:
```
include_directories(
    include 
    ${Boost_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS}
    ...
)
```

### Add Target Executables and Libraries
The add_executable() CMake function is used to specify an **executable target** to build.
```
add_executable(myProgram src/main.cpp src/some_file.cpp src/another_file.cpp)
```
This will build a target executable called myProgram which is built from 3 source files: src/main.cpp, src/some_file.cpp and src/another_file.cpp.

The add_library() CMake function is used to specify **library target** to build. 
By default catkin builds shared libraries. 
```
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
```

The target_link_libraries() CMake function is used to which libraries an executable target links against. This is done typically after an add_executable() call.

Syntax: 
```
target_link_libraries(<executableTargetName>
   <lib1> 
   <lib2> 
   ... 
   <libN>
) 
```
Example:
```
add_executable(foo src/foo.cpp) 
add_library(moo src/moo.cpp) 
target_link_libraries(foo moo) -- This links foo against libmoo.so
```

## Messages, Services and Action Targets
[Document](http://docs.ros.org/kinetic/api/catkin/html/howto/format2/building_msgs.html)

For CMake, find the catkin packages for message_generation and any messages, services or actions you depend on:
```
find_package(catkin REQUIRED COMPONENTS   
  message_generation 
  std_msgs)
```

For building actions, include actionlib_msgs among the dependencies:
```
find_package(catkin REQUIRED
             COMPONENTS
             actionlib_msgs
             message_generation
             std_msgs)
```

Next, list your message definitions:
```
add_message_files(DIRECTORY msg
                  FILES
                  YourFirstMessage.msg
                  YourSecondMessage.msg
                  YourThirdMessage.msg)
```
Then, generate all your message, service and action targets with this command:
```
generate_messages(DEPENDENCIES std_msgs)
```

Make sure the catkin_package() command declares your message, service and action dependencies for other packages:
```
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
```
These macros must come **BEFORE** the catkin_package() macro in order for generation to work correctly.
```
find_package(catkin REQUIRED COMPONENTS ...) 
add_message_files(...) add_service_files(...) add_action_files(...) generate_messages(...) catkin_package(...)
```
A good ROS practice is to collect related messages, services and actions into a separate package with no other API. That simplifies the package dependency graph.

However, you can provide scripts and programs with the message package. If you do, message generation targets need to be built before any programs that depend on them. Every target that directly or indirectly uses one of your message headers must declare an explicit dependency:
```
add_dependencies(your_program ${${PROJECT_NAME}_EXPORTED_TARGETS})
```
If your build target also uses message or service headers imported from other catkin packages, declare those dependencies similarly:
```
add_dependencies(your_program ${catkin_EXPORTED_TARGETS})
```
