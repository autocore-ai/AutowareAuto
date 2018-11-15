How to write integration tests {#integration-testing}
========

[TOC]

# Goals {#integration-tests-1}


In this article we will demonstrate the purpose of integration tests, how to write
integration tests, how to run these tests and how to evaluate results.

# Quick reference {#integration-tests-refs}


1. [colcon](https://github.com/ros2/ros2/wiki/Colcon-Tutorial) is used to build and run test
2. `integration_tests` is used to specify tests in CMakeLists.txt
3. [pytest](https://docs.pytest.org/en/latest/) is used to eventually execute the test,
generate jUnit format test result and evaluate the result
4. [unit testing](@ref how-to-write-tests-and-measure-coverage) describes testing big picture

# What is integration test {#integration-tests-what}


Integration test is defined as the phase in software test in which individual software
modules are combined and tested as a group. Integration test occurs after unit test and before validation
test. Integration testing takes as its input modules that have been unit tested, groups
them in larger aggregates, applies tests defined in an integration test plan to those
aggregates, and delivers as its output the integrated system ready for system testing
([Wikipedia](https://en.wikipedia.org/wiki/Integration_testing)).

# Why integration test {#integration-tests-why}


Integration test determines if independently developed units of software work correctly
when they are connected to each other. In ROS 2 these units are called
nodes and integration tests can find following errors:

- It can find incompatible interaction between nodes, such as non-matching topics,
different message types or incompatible QoS settings
- It also helps reveal edge cases that were not touched with unit test, such as
critical timing issue, network communication delay, disk I/O failure, and many other problems
that would occur in production environment
- By using tools like `stress` and `udpreplay`, performance of nodes can be tested with real data
or under high CPU/memory load where eg., malloc failures can be detected

With ROS 2 you can program very complex autonomous driving applications with the large number
of nodes. Therefore, a lot of effort has been made to provide the integration test framework
that helps the developer to test integration of their nodes.

# Integration test framework architecture {#integration-tests-architecture}


A typical integration test has three components:

1. A series of executables with arguments that work together and generate some outputs
2. A series of expected outputs whose content must be found or matched for each executable
3. A launcher that starts the tests, checks the expected outputs, and decides if the test passes

An example would be a talker and a listener that communicate via ROS 2 message. Code of the
talker and listener can be found
[here](https://github.com/ros2/demos/tree/master/demo_nodes_cpp/src/topics).
Talker sends a message with incrementing index which, ideally,
is consumed by the listener. However, this is not always the case. Errors could happen
when the developers misspell the topic name, use different message type or incompatible QoS
settings.

In this case, the three aforementioned components are:

1. executables: talker and listener, with arguments like publishing periodic message
2. expected outputs: two regular expressions that are expected to be found in the stdout of
talker and listener
3. launcher: a launching script in python that invokes executables and checks expected output

\note Regular expression(regex) is used as the expected outputs pattern. More information about
regex can be found in [RegExr](https://regexr.com/).

Launcher starts talker and listener in sequence and periodically checks the outputs. If the regex
pattern are found in the output, launcher exits with a return code of 0 and marks the test
as successful. Otherwise, the integration test is marked as failed.

The sequence of executables sometimes matters. If `exe_b` depends on resources created
by `exe_a`, the launcher must start them in the right order.

Some nodes are designed to run indefinitely. So the launcher is able to terminate the
executables when all the output patterns are satisfied, or after a certain amount of time.
Otherwise, the executables have to use a `runtime` argument.

# Integration test framework {#integration-tests-framework}


In this section we will provide examples for how to use `integration_tests` framework.
The architecture of `integration_tests` framework is shown in the diagram.

![integration_test architecture](@ref process_of_apex_integration_tests.png)

## Integration test with single executable {#integration-tests-single}


We will first start with the simplest scenario. A package named `my_cool_pkg` was created under
workspace directory with [this helper](https://gitlab.com/AutowareAuto/AutowareAuto/tree/master/src/tools/autoware_create_pkg).

`my_cool_pkg` has an executable that prints `Hello World` to stdout. We'd like to test if it
works as expected. Here are steps to add integration test:

1. Create a file `~/workspace/src/my_cool_pkg/test/expected_outputs/my_cool_pkg_exe.regex` with
   content `Hello\sWorld`. This is the regular expression that you want to test against
   the stdout of your executable

2. Under the `BUILD_TESTING` code block, call `integration_tests` to add the test
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

3. Build `~/workspace/`

4. Run the integration test
   ```bash
   ade$ colcon test --merge-install --packages-select my_cool_pkg --ctest-args -R integration
   ...
   Starting >>> my_cool_pkg
   Finished <<< my_cool_pkg [4.79s]

   Summary: 1 package finished [6.30s]

   ```

   \note Use `--ctest-args -R integration` to run integration tests only.

`colcon test` parses the package tree, looks for correct build directory and runs the test
script. `colcon test` generates a jUnit format test result for this test.

By default `colcon test` gives a very brief test report. To get detailed statistics about
this test, refer to the following directories/files:

1. `~/workspace/log/latest_test/my_cool_pkg` is the directory that holds all the commands,
   ctest stdout, and stderr

   \note This directory only contains output of `ctest`, not the output
   of tested executables.

   Files under this directory include:

   1. `command.log` contains all the test commands, including their working directory, executables,
      arguments
   2. `stderr.log` contains the standard error of `ctest`
   3. `stdout.log` contains the standard output of `ctest`

2. The stdout of the tested executable is stored in
   `~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/my_cool_pkg_exe_integration_test.xunit.xml`
   in jUnit format
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
   is tested so `i` starts from 0. Note that `test_executable_0` prints
   `Hello World` to stdout, which is captured by launcher. The output matches the regex
   `Hello\sWorld` specified in the expected output file. The launcher then broadcasts a
   SIGINT to all the test executables and marks the test as successful. Otherwise, the
   integration test fails.

\note SIGINT is broadcast only if the output of the **last** executable matches its regex.

## How exactly `integration_tests` framework works under the hood

`integration_tests` is a wrapper of [ament_add_pytest_test]
(https://github.com/ament/ament_cmake/blob/master/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake).
It receives organized commands, test name, and expected outputs as arguments.

Its arguments include:
- `TESTNAME`: optional, a string of integration test name
- `EXPECTED_OUTPUT_DIR`: required, an absolute path where the expected output files are stored.
  Files under this directory will be copied to `~/workspace/build/package-name/expected_outputs/`
  with suffix added
- `COMMANDS`: required, a series of `:::` separated commands.
  Eg. `talker_exe --topic TOPIC:::${LISTENER_EXE} --topic TOPIC`
- `SUFFIX`: optional, a string that will be appended to all the regex file name

When `integration_tests` is called by `my_cool_pkg` with correct arguments:

1. It first splits `COMMANDS` using `:::`, so that
   `talker_exe --topic TOPIC:::${LISTENER_EXE} --topic TOPIC` becomes a list
   `talker_exe --topic TOPIC;listener_exe --topic TOPIC`

2. Next it splits each command by space and extracts the executables and arguments

3. It replaces each executable with [generator_expression]
   (https://cmake.org/cmake/help/latest/manual/cmake-generator-expressions.7.html)
   `$<TARGET_FILE:${executable}>`. In this way, cmake is able to find the path to the executable

4. If `EXPECTED_OUTPUT_DIR` is specified, it copies files under `EXPECTED_OUTPUT_DIR` to
   `~/workspace/build/my_cool_pkg/expected_outputs/` with proper suffix

5. It generates a list of expected regex files, each of which has the following format:
   `~/workspace/build/my_cool_pkg/expected_outputs/[namespace__]executable[SUFFIX]`

6. It configures a python script `test_executables.py.in` that uses ROS launch to start
   all processes and checks the outputs



## Integration test with multiple executables {#integration-tests-multiple}


In the `my_cool_pkg` example, only one executable is added to integration test. But most of the
times, we'd like to test the interaction between several executables. Suppose `my_cool_pkg` has
two executables, a talker and a listener which communicate with each other via ROS 2 topic.

The launcher starts the talker and listener at the same time. Talker starts incrementing the
index and sending it to the listener. Listener receives the index and prints it to the stdout.
The passing criteria is if listener receives indices 10, 15 and 20.

Here are the steps to add multiple-executable integration test:

1. Create two files
   - `~/workspace/src/my_cool_pkg/test/expected_outputs/talker_exe.regex` with content `.*`
   - `~/workspace/src/my_cool_pkg/test/expected_outputs/listener_exe.regex` with content
```bash
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
   Here, `:::` is used as delimiter of different executables. `integration_tests` parses
   the executables, arguments, and composes a correct test python script.

3. Build `~/workspace/`

4. Run the integration test
   ```bash
   ade$ colcon test --merge-install --packages-select my_cool_pkg --ctest-args -R integration
   Starting >>> my_cool_pkg
   Finished <<< my_cool_pkg [20.8s]

   Summary: 1 package finished [22.3s]
   ```

If everything is configured correctly, after 20 seconds, the integration test shall pass. Things
that happen behind this are similar to the single executable example. Launcher starts
talker and listener at the same time, and periodically checks the stdout of each executable.
Here, the regex of talker is `.*`, which always matches when the first output of talker
is captured by launcher. The regex of listener is 10, 15, and 20. After all entries in this
regex are matched, a SIGINT is sent to all commands and the test is marked as successful.

The locations of output files are the same with single executable example. Output of `ctest` is
is `~/workspace/log/latest_test/my_cool_pkg/`. Output of tested executables is stored in
`~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/` in jUnit format.

\note By the time SIGINT is sent, all the regex have to be successfully matched in the output.
Otherwise the test is marked as failed. Eg. if the regex for talker is `30`, the test will fail.

## Use executables from another package {#integration-tests-other-exe}


Sometimes an integration test needs to use executables from another package. Suppose
in `my_cool_pkg` you want to test `talker` and `listener` defined in `demo_nodes_cpp`. These two
executables must be exported by `demo_nodes_cpp` and then imported by `my_cool_pkg`. Also a
namespace must be added before `talker` and `listener` to indicate that executables are from
another package.

Here are steps to add integration test:

1. Add `<buildtool_depend>ament_cmake</buildtool_depend>` to
   `~/workspace/src/demo_nodes_cpp/package.xml`

2. In `~/workspace/src/demo_nodes_cpp/CMakeLists.txt`, export the executable target
   ```cmake
   find_package(ament_cmake REQUIRED)
   ament_export_interfaces(talker listener)
   ```
3. Create two regex files
   - `~/workspace/src/my_cool_pkg/test/expected_outputs/demo_nodes_cpp__talker.regex`
     with content `.*`
   - `~/workspace/src/my_cool_pkg/test/expected_outputs/demo_nodes_cpp__listener.regex`
     with content `20`

4. In `~/workspace/src/my_cool_pkg/package.xml`, add dependency
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

6. Build the workspace as before

7. Run the integration test
   ```bash
   ade$ colcon test --merge-install --packages-select my_cool_pkg --ctest-args -R integration
   ```

When `ament_export_interfaces(talker listener)` is called in `demo_nodes_cpp`, ament generates a
`demo_nodes_cppConfig.cmake` file which is used by `find_package`. The namespace in this file is
`demo_nodes_cpp`. Therefore, to use executable in `demo_nodes_cpp`, a namespace and `::` has to be
added. The format of an external executable is `namespace::executable --arguments`.
`integration_tests` function sets the regex file name as `namespace__executable.regex`.
One exception is that no namespace is needed for executable defined in the package that adds
this integration test.

## Add multiple integration tests in one package {#integration-tests-multiple-tests}


If `my_cool_pkg` has multiple integration tests added with the same executable but different
parameters, `SUFFIX` has to be used when calling `integration_tests`.

Suppose `my_cool_pkg` has an executable `say_hello` which prints `Hello {argv[1]}` to the screen.
We'd like to test if `say_hello` works with Alice and Box. Here are the steps to add multiple
integration tests:

1. Create two regex files
   - `~/workspace/src/my_cool_pkg/test/expected_outputs/say_hello_Alice.regex`
     with content `Hello\sAlice`
   - `~/workspace/src/my_cool_pkg/test/expected_outputs/say_hello_Bob.regex`
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
3. Build the workspace as before

4. Run the integration test
   ```bash
   ade$ colcon test --merge-install --packages-select my_cool_pkg --ctest-args -R integration
   ```

By specifying `SUFFIX`, `integration_tests` adds the correct suffix to the regex file path.
