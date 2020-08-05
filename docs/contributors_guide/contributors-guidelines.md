Guidelines and Best-Practices {#contributor-guidelines}
===========

[TOC]

# Contribution Workflow

1. [Create an issue](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/new?issue%5Bassignee_id%5D=&issue%5Bmilestone_id%5D=) defining your intended contribution.
  1. Use one of the provided templates [by selecting one from the drop-down list](https://docs.gitlab.com/ee/user/project/description_templates.html#using-the-templates).
  2. Select yourself in the `Assignee` field.
  3. If you have permissions to do so, assign to an appropriate milestone. If you do not have permissions, [mention](https://about.gitlab.com/blog/2016/03/08/gitlab-tutorial-its-all-connected/#mention-others-and-assign) a [maintainer](https://gitlab.com/groups/autowarefoundation/autoware.auto/committers/-/group_members) in the issue for milestone assignment.
  4. The issue template you choose will assign one appropriate label for the issue type (bug, discussion, feature, or improvement).
     Assign any additional labels from the available list that you feel are appropriate for the issue's status or other attributes.

2. Create a Fork
  1. For more information about the fork-and-pull model, see the [Develop in a Fork](@ref develop-in-a-fork) page.

3. Develop
  1. See the rest of this document for guidelines and best-practices on contributing to Autoware.Auto.

4. Create a Merge Request
  1. For more information about the fork-and-pull model, see the [Develop in a Fork](@ref develop-in-a-fork) page.

5. Merge Your Merge Request
  1. In order for a merge request to be merged to Autoware.Auto, it must meet the following criteria:
    - All discussions on the merge request must be resolved.
    - It must be approved by at least one maintainer.
    - CI jobs for the merge request must have passed successfully.
  2. If you have permissions, the "Merge" button will show up automatically on your merge request once the above criteria are met.
     If you do not have permissions, ask a maintainer to merge the merge request if the above criteria are met.
  3. If your merge request is from your fork to the AutowareAuto repository and you have to rebase prior to merging, make sure you notify a maintainer as soon as possible after your rebase.
     If another merge request is merged after you rebase, but before yours is merged, you will have to rebase again, which will trigger CI jobs to run again and slow down the merging of your merge request.


# System Dependencies and Target Environments

Autoware.Auto targets the environments and applications listed below. These may change in future versions.

## Target Platforms

- `amd64` / `x86_64` (Intel/AMD)
- `arm64` / `aarch64` / `arm64v8` (ARM v8, 64-bit)

## Target Operating Systems

- Ubuntu 18.04 LTS

## Target ROS Versions

- ROS Dashing

## Target System Dependencies and Versions

See [REP-2000's entry for ROS Dashing](https://www.ros.org/reps/rep-2000.html#dashing-diademata-may-2019-may-2021).


# Guidelines for General Code Development

Only C++14 and below is allowed for functional code.
Python 3.5+ and Bash are allowed for tooling.
CMake is the preferred build system.
Deviations need to be approved by the maintainers.

The requirements for C++14 and Python 3.5+ align with compiler and tooling support found in ROS Dashing targeted versions.
This may change in the future as new OS or ROS environments are targeted.

## Committing

Developers should commit and push regularly to the server to avoid data loss.

## IDEs

No specific IDE is required to develop software for Autoware.Auto.
However, we recommend using an IDE that has good support for ROS 2, such as Atom, Visual Studio Code or CLion.
If you plan to use the ADE (which is currently required), see the [Installation](@ref installation) guide.

## System-level Function Calls

It is preferred to use cross-platform solutions for system-level function calls whenever possible.
While the C++ standard library should be used for as many tasks as possible, some functions (such as `std::filesystem`) are not available in C++14 in cross-platform implementations.
This usually means utilizing libraries like [`asio`](https://think-async.com/Asio/index.html) for networking tasks and [`a std::filesystem shim`](https://github.com/gulrak/filesystem) for filesystem navigation is preferred to creating platform-specific implementations.

## Documentation

To check that the code is properly documented and all documents are correctly linked, you can run `AutowareAuto/docs/.doxygen/build.py`.
The generated documentation can be found in `AutowareAuto/docs/_build/html/index.html`.

## Formatting

Autoware.Auto follows ROS recommendations for code style and formatting.
See the [Coding Style and Language Versions entry for C++](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/#id3) or the [Coding Style and Language Versions entry for Python](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/#python) for more information.
We strictly enforce these guidelines using linters provided with `ament`.
All packages should have the following in their `package.xml` files:

```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
```

In addition, the following should be in the package's CMakeLists.txt (extended with other tests):

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```

## Code Coverage

For [code coverage](https://en.wikipedia.org/wiki/Code_coverage), we use the popular [GNU tool `lcov`](http://ltp.sourceforge.net/coverage/lcov.php) for generating statistics about line coverage.
For every merge request, we run our testsuite and report the percentage of lines covered.
We aim for a 100% line coverage and continuously improve our testsuite to achieve that number.
In particular, we do not accept changes that reduce the coverage value.
If a merge request has a lower line coverage than `master`, we will request the contributor to add more tests.

Instructions for generating a coverage report can be found here: \ref how-to-write-tests-and-measure-coverage-coverage.

For information about writing unit or integration tests, see the [Writing Unit Tests in ROS](#unit-tests-in-ros) and [Writing Integration Tests in ROS](#integration-tests-in-ros) sections.

## Resources

- [cppreference.com](https://en.cppreference.com/w/)
- [C++ Core Guidelines](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)


# Guidelines for ROS Development

In general, Autoware.Auto follows the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/) for contributions, except where noted.
Some special items of note which are not described in the ROS 2 Developer Guide are listed below.

## Creating a New Package

Basic instructions for creating a new ROS 2 package can be found [in this tutorial](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/#create-your-own-package).
In Autoware.Auto, much of the boilerplate code can be automatically generated by utilizing the `autoware_auto_create_pkg` tool.

For more information on using the tool, see \ref autoware_auto_create_pkg-package-design.

## Writing Unit Tests in ROS {#unit-tests-in-ros}

All good code contributions come with tests - both unit tests and integration tests.
The [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#testing) has a pretty good overview of testing requirements.
In ROS 2, `gtest` is the primary testing framework and the CMake macro `ament_add_gtest` is the mechanism for adding tests.
<a href="https://github.com/bponsler/ros2-support/blob/master/tutorials/unit-testing.md">This guide</a> is a very good starting point for creating tests.
More information about the `ament_add_gtest` macro can be found in [this ROS tutorial](https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/#testing).
The basic syntax for `gtest` tests is covered in the <a href="https://github.com/google/googletest/blob/master/googletest/docs/primer.md">Googletest Primer</a>.
In addition, you'll find examples all over the Autoware.Auto codebase of tests you can use as templates.

## Writing Integration Tests in ROS {#integration-tests-in-ros}

[launch_testing](https://github.com/ros2/launch/tree/master/launch_testing#launch_testing), introduced in ROS 2 Dashing, is the recommended mechanism for writing integration tests.
Any package that has an executable will be required to have at least one accompanying integration test.
The purpose of the integration test is to start the executable and ensure that it processes any input data correctly and produces outputs according to its specification.
For more information on integration testing, [see this article on integration testing fundamentals](http://softwaretestingfundamentals.com/integration-testing/).

## Package Types

TBD

## 3-Tier Development Pattern

In all but the most trivial utilities, it is best to implement a code pattern with *at least* 3 tiers of abstraction.
The three basic tiers would look something like:

1. A "core," pure C++ class which performs all basic mathematical and utility functions which are not ROS-related.

   This class may use ROS utilities such as logging or message structures, but such use must be justified in terms of why it cannot be done via the class's external interface (e.g. the enclosing node uses information obtained via the class's external interface to populate log messages).
2. A "ROS Node" class which inherits from rclcpp::Node or a subclass, handles all ROS-specific functions, and instantiates the class defined in 1.
3. A "ROS Executable" which contains the main function, creates an instance of the class defined in 2, and handles provision of execution time in some way (e.g. through calling rclcpp::spin()).

This design pattern helps to promote seperation-of-concerns and code re-use.
There are some trivial cases where a simple ROS Node which does not require a "core" are acceptable but these should be the exception, not the rule.

## Naming in Autoware.Auto

The [Naming Guidelines](@ref autoware-common-naming-guidelines) provide for standard, reliable naming and namespacing conventions which should be used in all Autoware.Auto packages.

## On Topics and Parameters

In most cases, topics should receive a default name in code and be remapped if needed.
Providing topic names as ROS parameters is an anti-pattern, with few exceptions.

### Parameter File Syntax

To avoid the need to change parameter files based on the namespacing or node name of a node, use the "double-star" syntax. e.g.:

```yaml
/**:
  ros__parameters:
    param1: value
```

The above parameter file can be passed to any node regardless of namespace or name and the parameters will populate those of the node if the declared parameters match those in the file.

## ROS Components

As of ROS Dashing, the recommended way to write Nodes in ROS 2 is using Components.
For more information about components and their use, see [the ROS Composition Guide](https://index.ros.org/doc/ros2/Tutorials/Composition/).
To implement your node as a Component, it must conform to the items below (using "ListenerNode" as an example):

- Must inherit from `rclcpp::Node` or a subclass (such as `rclcpp::LifecycleNode`)
- Must use a single-argument constructor in the form of:

```c++
namespace composition_example
{
class ListenerNode: public rclcpp::Node {
  ListenerNode(const rclcpp::NodeOptions & options)
  : Node("listener", options)
  {
    ...
  }
}
}  // namespace composition_example
```

- Must contain a registration macro and header in a single translation unit. For example, the following at the bottom of your `cpp` file would suffice:

```c++
// Insert at bottom of translation unit, e.g. listener_node.cpp
#include <rclcpp_components/register_node_macro.hpp>
// Use fully-qualified name in registration
RCLCPP_COMPONENTS_REGISTER_NODE(composition_example::ListenerNode)
```

- Must compile the components as a shared library and register them in your CMakeLists.txt file.
  The following is a minimal CMakeLists.txt file which uses the recommended `ament_cmake_auto` macros, registers a single component, builds a stand-alone node which uses the component, and exports it as a dependency for downstream packages:

```cmake
cmake_minimum_required(VERSION 3.5)
project(composition_example)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(listener_node SHARED src/listener_node.cpp)
autoware_set_compile_options(listener_node)
rclcpp_components_register_nodes(listener_node "composition_example::ListenerNode")

ament_auto_add_executable(listener_node_exe src/listener_main.cpp)
autoware_set_compile_options(listener_node_exe)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
```

- Must depend on the `rclcpp_components` package.
  The following is a minimal package.xml file to go with the above CMakeLists.txt example:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>composition_example</name>
  <version>0.0.1</version>
  <description>Example of node composition</description>
  <maintainer email="my.email@example.com">The Autoware Foundation</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake_auto</buildtool_depend>
  <buildtool_depend>autoware_auto_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```


## Building, Running, and Testing

### How to Build Autoware.Auto {#building-autoware-auto}

ROS 2 uses the `colcon` build system. For more information and details about options and flags, see [the colcon documentation](https://colcon.readthedocs.io/en/released/user/quick-start.html).
To build the entire AutowareAuto codebase, do the following:

```bash
ade enter
cd ~/AutowareAuto
colcon build
```

To just build a single package:

```bash
colcon build --packages-select <package_name>
```

To build a single package and all of its dependencies recursively:

```bash
colcon build --packages-up-to <package_name>
```

### How to Run an Executable

To run an executable from Autoware.Auto, there are two options:

1. Run a pre-built executable from the AutowareAuto volume in ADE:

```bash
ade enter
source /opt/AutowareAuto/setup.bash
ros2 run <package_name> <executable_name>
```

2. Run an executable from a custom package that you added to Autoware.Auto:
  1. Build the package or the entire stack using the [instructions above](#building-autoware-auto).
  2. Run:

```bash
cd ~/AutowareAuto
source install/setup.bash
ros2 run <package_name> <executable_name>
```

### How to Run Tests

First, build your package or the entire stack using the [instructions above](#building-autoware-auto).
After this, run:

```bash
colcon test
colcon test-result --verbose
```

The first command will run the tests attached to the packages in your workspace.
The second command gives you detailed output from the tests on which ones passed and which failed.

## Resources

- [rclcpp_components in Dashing Diademata Release Notes](https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata/#rclcpp-components)
