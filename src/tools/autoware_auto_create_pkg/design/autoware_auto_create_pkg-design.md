autoware_auto_create_pkg {#autoware_auto_create_pkg-package-design}
===========

This is the design document for the `autoware_auto_create_pkg` package.


# Purpose

To help ensure consistency among packages, Autoware.Auto provides this package creation script that provides default build settings and boilerplate code for use when creating a new ROS 2 package.

## Differences from ROS 2 package creation tools

The below sections describe a couple differences of note between this package creation script and [those one provided by ROS 2](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/#create-your-own-package).

### ament_cmake_auto

The `autoware_auto_create_pkg` script sets up packages to use [ament_cmake_auto](https://github.com/ament/ament_cmake/tree/master/ament_cmake_auto) rather than [ament_cmake](https://github.com/ament/ament_cmake/tree/master/ament_cmake).
The `ament_cmake_auto` library contains CMake macros like `ament_auto_add_library` which automate many boilerplate tasks like targeting dependencies.
For example, if you use `ament_auto_find_build_dependencies()`, it will automatically `find_package()` all the build-related dependencies in package.xml.
Then, if you use the `ament_auto_` forms of `add_library` or `add_executable`, it will automatically `ament_target_dependencies()` for all of the found ones.

### Visibility control

The `autoware_auto_create_pkg` script gives finer grained control over library linkage via the `visibility_control.hpp` header it includes in each package.

The purpose of visibility control is to get library symbol visibility in a consistent state on different platforms (i.e. Windows uses the equivalent of `-fvisibility=hidden`).
As an example, ROS2 code does this.
This means to get good cross-platform support, we need to prepend i.e. functions with:

```
FOO_PUBLIC int my_func();

class FOO_PUBLIC bar  // all symbols visible by default
{
public:
  void bazzle();  // public visibility

private:
  FOO_LOCAL gaddle();  // LOCAL visibility -> can't be linked to
};
```

This also gives us the ability to mark symbols (classes, functions, methods, etc) as local so that external users can't link to them.
There is also [some argument](https://gcc.gnu.org/wiki/Visibility) that it improves security and linking/loading times.

> NOTE: Default visibility is `LOCAL`, so be sure to mark any classes or functions as `PUBLIC` that you want to be able to link against.

## Usage

To use the tool, run the following:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ cd AutowareAuto/src/<path_to_folder_containing_your_new_package>
ade$ ros2 run autoware_auto_create_pkg main.py --pkg-name PKG_NAME --maintainer MAINTAINER --email EMAIL --description DESCRIPTION
```

In the above commands, replace `<path_to_folder_containing_your_new_package>` with the path to the folder where you want to create your package.
Alternatively, add the `--destination <path_to_folder_containing_your_new_package>` argument to generate the package in a different location than the current working directory.
Also replace the flag values in ALL_CAPS with appropriate values for your new package.
Once your new package has been created, you will need to clean up the specifics (license, dependencies, design document, etc.).

To obtain more details on the command-line usage, call:

```
ade$ ros run autoware_auto_create_pkg main.py --help
```

Once the package has been created with for example `--pkg-name foo`, build and test it.

```
colcon build --packages-select foo
colcon test  --packages-select foo
```

Then follow these steps to run the node from a launch file:

```
source install/setup.bash
ros2 launch foo foo.launch.py
```

The output should be similar to:

```
[component_container-1] Hello World
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/foo_node' in container '/foo_container'
```

Alternatively run the node executable directly:

```
source install/setup.bash
ros2 run foo foo_node_exe
```

## References / External links
<!-- Optional -->

- Basic instructions for creating a new ROS 2 package can be found [in this tutorial](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/#create-your-own-package).
- Basic instructions regarding [ROS2 composition](https://index.ros.org/doc/ros2/Tutorials/Composition/)

## Related issues
<!-- Required -->

- https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/462
- https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/561
- https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/624
