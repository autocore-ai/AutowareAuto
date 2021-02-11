How to Write Integration Tests {#integration-testing}
========

[TOC]

# Goals {#integration-tests-1}

This article motivates developers to adopt integration testing by explaining how to write, run,
and evaluate the results of integration tests.


# Quick reference {#integration-tests-refs}

1. [colcon](https://github.com/ros2/ros2/wiki/Colcon-Tutorial) is used to build and run test
3. [pytest](https://docs.pytest.org/en/latest/) is used to eventually execute the test,
generate jUnit format test result and evaluate the result
4. [unit testing](@ref how-to-write-tests-and-measure-coverage) describes testing big picture


# Introduction {#how-to-write-integration-tests-introduction}

This article motivates developers to adopt integration testing by explaining how to write, run,
and evaluate the results of integration tests.


## Quick reference {#integration-tests-quick-reference}

1. [colcon](https://index.ros.org/doc/ros2/Colcon-Tutorial/) is used to build and run the tests
2. `integration_tests` is used to specify tests in the CMakeLists.txt files
3. [pytest](https://docs.pytest.org/en/latest/) is used to eventually execute the test,
generate jUnit format test result, and evaluate the result


# Integration testing {#how-to-write-integration-tests-integration-testing}

An integration test is defined as the phase in software testing where individual software
modules are combined and tested as a group. Integration tests occur after unit tests, and before
validation tests.

The input to an integration test is a set of independent modules that have been unit tested. The set
of modules are tested against the defined integration test plan, and the output is a set of
properly integrated software modules that are ready for system testing.


# Value of integration testing {#how-to-write-integration-tests-value-of-integration-testing}

Integration tests determine if independently developed software modules work correctly
when the modules are connected to each other. In ROS 2, the software modules are called
nodes.

Integration tests help to find the following types of errors:

- Incompatible interaction between nodes, such as non-matching topics, different message types, or
incompatible QoS settings
- Reveal edge cases that were not touched with unit tests, such as a critical timing issue, network
communication delay, disk I/O failure, and many other problems that can occur in production
environments
- Using tools like `stress` and `udpreplay`, performance of nodes is tested with real data
or while the system is under high CPU/memory load, where situations such as `malloc` failures can be
detected

With ROS 2, it is possible to program complex autonomous driving applications with a large number
of nodes. Therefore, a lot of effort has been made to provide an integration test framework that
helps developers test the interaction of ROS2 nodes.


# Integration test framework architecture {#how-to-write-integration-tests-integration-test-framework-architecture}

A typical integration test has three components:

1. A series of executables with arguments that work together and generate outputs
2. A series of expected outputs that should match the output of the executables
3. A launcher that starts the tests, compares the outputs to the expected outputs,
and determines if the test passes


## A simple example {#how-to-write-integration-tests-a-simple-example}

The simple example consists of a `talker` node and a `listener` node. Code of the `talker` and
`listener` is found in the
[ROS 2 demos](https://github.com/ros2/demos/tree/master/demo_nodes_cpp/src/topics) repository.

The `talker` sends messages with an incrementing index which, ideally,
are consumed by the `listener`.

The integration test for the `talker` and `listener` consist of
the three aforementioned components:

1. Executables: `talker` and `listener`
2. Expected outputs: two regular expressions that are expected to be found in the `stdout` of the
executables:
   - `Publishing: 'Hello World: ...'` for the `talker`
   - `I heard: [Hello World: ...]` for the `listener`
3. Launcher: a launching script in python that invokes executables and checks for the expected
output

\note
Regular expressions (regex) are used to match the expected output pattern. More information about
regex can be found at the [RegExr](https://regexr.com/) page.

The launcher starts `talker` and `listener` in sequence and periodically checks the outputs. If the
regex patterns are found in the output, the launcher exits with a return code of `0` and marks the
test as successful. Otherwise, the integration test returns non-zero and is marked as a failure.

The sequence of executables can make a difference during integration testing. If `exe_b` depends on
resources created by `exe_a`, the launcher must start executables in the correct order.

Some nodes are designed to run indefinitely, so the launcher is able to terminate the
executables when all the output patterns are satisfied, or after a certain amount of time.
Otherwise, the executables have to use a `runtime` argument.


# Integration test framework {#how-to-write-integration-tests-integration-test-framework}

This section provides examples for how to use the `integration_tests` framework. The
architecture of `integration_tests` framework is shown in the diagram below.

![integration_test architecture](@ref process_of_apex_integration_tests.png)


## Integration test with a single executable {#how-to-write-integration-tests-integration-test-with-a-single-executable}

The simplest scenario is a single node. Create a package named `my_cool_pkg` in the `~/workspace`
directory; it's recommended to use the
[package creation tool](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/tree/master/src/tools/autoware_auto_create_pkg).

`my_cool_pkg` has an executable that prints `Hello World` to `stdout`. Follow the steps below to
add an integration test:

1. Create a file `~/workspace/src/my_cool_pkg/test/expected_outputs/my_cool_pkg_exe.regex` with the
   content `Hello\sWorld`
       - The string in the file is the regular expression to test against the `stdout` of the
       executable
2. Under the `BUILD_TESTING` code block, add a call to `integration_tests` to add the test
   ```cmake
   set(MY_COOL_PKG_EXE "my_cool_pkg_exe")
   add_executable(${MY_COOL_PKG_EXE} ${MY_COOL_PKG_EXE_SRC} ${MY_COOL_PKG_EXE_HEADERS})
   ...
   find_package(integration_tests REQUIRED)
   integration_tests(
     EXPECTED_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/test/expected_outputs/"
     COMMANDS
     "${MY_COOL_PKG_EXE}"
   )
   ...
   ```
3. Build `~/workspace/`, or just the `my_cool_pkg` package, using `colcon`:
   ```{bash}
   $ ade enter
   ade$ cd ~/workspace/
   ade$ colcon build --merge-install --packages-select my_cool_pkg
   ```
4. Run the integration test
   ```{bash}
   $ ade enter
   ade$ cd ~/workspace/
   ade$ colcon test --merge-install --packages-select my_cool_pkg --ctest-args -R integration
   ...
   Starting >>> my_cool_pkg
   Finished <<< my_cool_pkg [4.79s]

   Summary: 1 package finished [6.30s]

   ```

\note
Use `--ctest-args -R integration` to run integration tests only.

`colcon test` parses the package tree, looks for the correct build directory, and runs the test
script. `colcon test` generates a jUnit format test result for the integration test.

By default `colcon test` gives a brief test report. More detailed information exists in
`~/workspace/log/latest_test/my_cool_pkg`, which is the directory that holds the directories
`ctest`, `stdout`, and `stderr` output. Note that these directory only contains output of `ctest`,
not the output of tested executables.

1. `command.log` contains all the test commands, including their working directory, executables,
arguments
2. `stderr.log` contains the standard error of `ctest`
3. `stdout.log` contains the standard output of `ctest`


The `stdout` of the tested executable is stored in the file
`~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/my_cool_pkg_exe_integration_test.xunit.xml`
using jUnit format:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<testsuite errors="0" failures="0" name="pytest" skips="0" tests="1" time="3.121">
 <testcase classname="my_cool_pkg.build.my_cool_pkg.my_cool_pkg_exe_integration_test_Debug"
   file="~/workspace/build/my_cool_pkg/my_cool_pkg_exe_integration_test_Debug.py"
   line="36" name="test_executable" time="3.051783800125122">
   <system-out>(test_executable_0) pid 21242:
     ['~/workspace/build/my_cool_pkg/my_cool_pkg_exe'] (all >; console, InMemoryHandler: test_executable_0)
     [test_executable_0] Hello World
     [test_executable_0] signal_handler(2)
     (test_executable_0) rc 27
     () tear down
   </system-out>
 </testcase>
</testsuite>
```

`test_executable_i` corresponds to the `(i+1)th` executable. In this case, only one executable
is tested so `i` starts from `0`. Note that `test_executable_0` prints
`Hello World` to `stdout`, which is captured by the `launcher`. The output matches the regex
`Hello\sWorld` specified in the expected output file. The `launcher` then broadcasts a
`SIGINT` to all the test executables and marks the test as successful. Otherwise, the
integration test fails.

\note
`SIGINT` is broadcast only if the output of the **last** executable matches its regex.

For detailed information about how `integration_tests` operates, see [the Q&A]
(@ref how-to-write-integration-tests-how-does-integration-tests-work) section below.


## Integration test with multiple executables {#how-to-write-integration-tests-integration-test-with-multiple-executables}

In the `my_cool_pkg` example, only one executable is added to the integration test. Typically,
the goal is to test the interaction between several executables. Suppose `my_cool_pkg` has
two executables, a `talker` and a `listener` which communicate with each other with a ROS2
topic.

The `launcher` starts the `talker` and `listener` at the same time. The `talker` starts incrementing
the index and sending it to the `listener`. The `listener` receives the index and prints it to
`stdout`. The passing criteria for the test is is if `listener` receives the indices `10`, `15,` and
`20`.

Here are the steps to add multiple-executable integration tests:

1. Create two files
    1. `~/workspace/src/my_cool_pkg/test/expected_outputs/talker_exe.regex` with content `.*`
    2. `~/workspace/src/my_cool_pkg/test/expected_outputs/listener_exe.regex` with content
```{bash}
   10
   15
   20
```
2. Under the `BUILD_TESTING` code block, call `integration_tests` to add the test
```cmake
...
find_package(integration_tests REQUIRED)
integration_tests(
 EXPECTED_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/test/expected_outputs/"
 COMMANDS
 "talker_exe --topic TOPIC:::listener_exe --topic TOPIC"
)
...
```
    1. The character set `:::` is used as delimiter of different executables
    2. `integration_tests` parses the executables, arguments, and composes a valid test python
    script
    3. More information about the python script can be found in the
    [Q&A](@ref how-to-write-integration-tests-how-does-integration-tests-work) section
3. Build `~/workspace/`, or just the `my_cool_pkg` package, using `colcon`:
```{bash}
$ ade enter
ade$ cd ~/workspace/
ade$ colcon build --merge-install --packages-select my_cool_pkg
```
4. Run the integration test
```{bash}
$ ade enter
ade$ cd ~/workspace/
ade$ colcon test --merge-install --packages-select my_cool_pkg --ctest-args -R integration
Starting >>> my_cool_pkg
Finished <<< my_cool_pkg [20.8s]

Summary: 1 package finished [22.3s]
```

When the environment is properly configured, after 20 seconds, the integration test shall pass.
Similar to the single node example, the `launcher` starts the `talker` and `listener` at the same
time. The `launcher` periodically checks the `stdout` of each executable.

The regex of `talker` is `.*`, which always matches when the first output of `talker`
is captured by launcher. The regex of `listener` is `10`, `15`, and `20`. After all entries in this
regex are matched, a SIGINT is sent to all commands and the test is marked as successful.

The locations of output files are the same with single executable example. Output of `ctest` is
is `~/workspace/log/latest_test/my_cool_pkg/`. Output of tested executables is stored in
`~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/` in jUnit format.

\note
By the time `SIGINT` is sent, all the regex have to be successfully matched in the output.
Otherwise the test is marked as failed. For example, if the regex for `talker` is `30`, the test
will fail.


## Use executables from another package {#how-to-write-integration-tests-use-executables-from-another-package}

Sometimes an integration test needs to use executables from another package. Suppose
`my_cool_pkg` needs to test with the `talker` and `listener` defined in `demo_nodes_cpp`. These two
executables must be exported by `demo_nodes_cpp` and then imported by `my_cool_pkg`.

When declaring the test, a namespace must be added before `talker` and `listener` to indicate that
executables are from another package.

Use the following steps to add an integration test:

1. Add `<buildtool_depend>ament_cmake</buildtool_depend>` to
`~/workspace/src/demo_nodes_cpp/package.xml`

2. In `~/workspace/src/demo_nodes_cpp/CMakeLists.txt`, export the executable target before
   calling `ament_package()`
   ```cmake
   install(TARGETS talker EXPORT talker
     DESTINATION lib/${PROJECT_NAME})
   install(TARGETS listener EXPORT listener
     DESTINATION lib/${PROJECT_NAME})
   find_package(ament_cmake REQUIRED)
   ament_export_interfaces(talker listener)
   ```
3. Create two regex files
    1. `~/workspace/src/my_cool_pkg/test/expected_outputs/demo_nodes_cpp__talker.regex`
     with content `.*`
    2. `~/workspace/src/my_cool_pkg/test/expected_outputs/demo_nodes_cpp__listener.regex`
     with content `20`

4. In `~/workspace/src/my_cool_pkg/package.xml`, add the dependency to `demo_nodes_cpp`
```cmake
<test_depend>demo_nodes_cpp</test_depend>
```
5. Under the `BUILD_TESTING` code block in `~/workspace/src/my_cool_pkg/CMakeLists.txt`, call
`integration_tests` to add the test
```cmake
...
find_package(integration_tests REQUIRED)
find_package(demo_nodes_cpp REQUIRED) # this line imports targets(talker) defined in namespace demo_nodes_cpp
integration_tests(
 EXPECTED_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/test/expected_outputs/"
 COMMANDS
 "demo_nodes_cpp::talker:::demo_nodes_cpp::listener" # format of external executable is namespace::executable [--arguments]
)
...
```
6. Build `~/workspace/`, or just the `my_cool_pkg` package, using `colcon`:
```{bash}
$ ade enter
ade$ cd ~/workspace/
ade$ colcon build --merge-install --packages-select my_cool_pkg
```
7. Run the integration test
```{bash}
$ ade enter
ade$ cd ~/workspace/
ade$ colcon test --merge-install --packages-select my_cool_pkg --ctest-args -R integration
```

When `ament_export_interfaces(talker listener)` is called in `demo_nodes_cpp`, ament generates a
`demo_nodes_cppConfig.cmake` file which is used by `find_package`. The namespace in this file is
`demo_nodes_cpp`. Therefore, to use executable in `demo_nodes_cpp`, a namespace and `::` has to be
added.

The format of an external executable is `namespace::executable --arguments`. The
`integration_tests` function sets the regex file name as `namespace__executable.regex`.
One exception is that no namespace is needed for executable defined in the package that adds
this integration test.


## Add multiple integration tests in one package {#how-to-write-integration-tests-add-multiple-integration-tests-in-one-package}

If `my_cool_pkg` has multiple integration tests added with the same executable but different
parameters, `SUFFIX` has to be used when calling `integration_tests`.

Suppose `my_cool_pkg` has an executable `say_hello` which prints `Hello {argv[1]}` to the screen.
Here are the steps to add multiple
integration tests:

1. Create two regex files
    1. `~/workspace/src/my_cool_pkg/test/expected_outputs/say_hello_Alice.regex`
     with content `Hello\sAlice`
    2. `~/workspace/src/my_cool_pkg/test/expected_outputs/say_hello_Bob.regex`
     with content `Hello\sBob`
2. Call `integration_tests` to add integration test
```cmake
integration_tests(
 EXPECTED_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/test/expected_outputs/"
 COMMANDS "say_hello Alice"
 SUFFIX "_Alice"
)
integration_tests(
 EXPECTED_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/test/expected_outputs/"
 COMMANDS "say_hello Bob"
 SUFFIX "_Bob"
)
```
3. Build `~/workspace/`, or just the `my_cool_pkg` package, using `colcon`:
```{bash}
$ ade enter
ade$ cd ~/workspace/
ade$ colcon build --merge-install --packages-select my_cool_pkg
```
4. Run the integration test
```{bash}
$ ade enter
ade$ cd ~/workspace/
ade$ colcon test --merge-install --packages-select my_cool_pkg --ctest-args -R integration
```

By specifying `SUFFIX`, `integration_tests` adds the correct suffix to the regex file path.


# Q&A

## How does integration_tests work {#how-to-write-integration-tests-how-does-integration-tests-work}

`integration_tests` is a wrapper of
[ament_add_pytest_test](https://github.com/ament/ament_cmake/blob/master/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake).
`integration_tests` receives organized commands, test name, and expected outputs as arguments.

The arguments include:

- `TESTNAME`: optional, a string of integration test name
- `EXPECTED_OUTPUT_DIR`: required, an absolute path where the expected output files are stored
    - Files under this directory will be copied to
    `~/workspace/build/package-name/expected_outputs/` with the suffix added
- `COMMANDS`: required, a series of `:::` separated commands
    - For example, `talker_exe --topic TOPIC:::${LISTENER_EXE} --topic TOPIC`
- `SUFFIX`: optional, a string that will be appended to the regex file name

When `integration_tests` is called by `my_cool_pkg` with correct arguments:

1. `integration_tests` splits `COMMANDS` using `:::`, so that
`talker_exe --topic TOPIC:::${LISTENER_EXE} --topic TOPIC` becomes a list:
`talker_exe --topic TOPIC;listener_exe --topic TOPIC`
2. Next `integration_tests` splits each command by a space, and extracts the executables and
arguments
3. `integration_tests` replaces each executable with [generator_expression]
(https://cmake.org/cmake/help/latest/manual/cmake-generator-expressions.7.html)
`$<TARGET_FILE:${executable}>`
    1. Using this method, cmake is able to find the path to the executable
4. If `EXPECTED_OUTPUT_DIR` is specified, `integration_tests` copies files under
`EXPECTED_OUTPUT_DIR` to `~/workspace/build/my_cool_pkg/expected_outputs/` with the proper suffix
5. `integration_tests` generates a list of expected regex files, each of which has the
following format: `~/workspace/build/my_cool_pkg/expected_outputs/[namespace__]executable[SUFFIX]`
6. `integration_tests` configures a python script `test_executables.py.in` that uses ROS launch
to start all processes and checks the outputs


## How are regex files prepared {#how-to-write-integration-tests-how-are-regex-files-prepared}

The simplest way to prepare a regex file corresponding to an executable is to put it under
`~/workspace/src/package-name/test/expected_outputs/` and pass this as `EXPECTED_OUTPUT_DIR` to
`integration_tests`. If this file needs to be generated dynamically, the upstream
package can create the file
`~/workspace/build/package  -name/expected_outputs/executable[suffix].regex` with the
expected regex. `integration_tests` doesn't care how this file is generated, as long as
it's in the correct place.


## How is the regex file handled {#how-to-write-integration-tests-how-is-the-regex-file-handled}

A regex file will first be delimited by a line break and then stored in a list. Every time the
`launcher` gets a callback from `stdout`, the launcher tries to match each regex in the list against
`stdout` and removes the regex that gets matched from the list.

When the regex list of the last executable becomes empty, a `SIGINT` is broadcast to all
executables. The test is marked as passed successfully if the regex lists of all executables are
empty, and a `SIGINT` is broadcast to all executables.
