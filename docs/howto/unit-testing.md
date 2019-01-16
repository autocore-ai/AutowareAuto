How to write tests and measure coverage {#how-to-write-tests-and-measure-coverage}
========

[TOC]

# Goals {#how-to-write-tests-and-measure-coverage-goals}

In this article we will motivate why testing of code is important and what kind of
testing we carry out in Autoware.Auto.

Furthermore, we will introduce how to write unit tests, how to run them, and how
to track code test coverage.


# Quick reference {#how-to-write-tests-and-measure-coverage-quick-reference}

1. [colcon](https://github.com/ros2/ros2/wiki/Colcon-Tutorial) is the tool of choice for building
and running tests
2. [ament_cmake](https://github.com/ament/ament_cmake) is useful to specify tests in CMake
3. Chris Hobbs' [*Embedded Software Development for Safety Critical Systems*](https://www.amazon.com/Embedded-Software-Development-Safety-Critical-Systems/dp/1498726704),
describes tests necessary for code running in safety critical environments
4. [ISO 26262 standard](https://www.iso.org/standard/51362.html) part 6 prescribes
how to test code in automotive systems
5. [SQLite](https://www.sqlite.org/testing.html) is a software project that has an impressive and
thoroughly described testing system


# Why testing {#how-to-write-tests-and-measure-coverage-why-testing}

Dynamic and static testing methods make Autoware.Auto reliable and robust, helping us to
perform anomaly detection and handling that would otherwise be difficult to find.
Through testing, in Autoware.Auto we can estimate the number of Heisenbugs, and find
and eliminate [undefined behaviours](https://blog.regehr.org/archives/1520) for
which C and C++ languages are known.

Dynamic analysis, simply called “testing” as a rule, means executing the code
while looking for errors and failures.

Static analysis means inspecting the code to look for faults. Static analysis is
using a program (instead of a human) to inspect the code for faults.

There are also formal verification methods (see the [book](https://www.amazon.com/Embedded-Software-Development-Safety-Critical-Systems/dp/1498726704),
Chapter 15), but note that they will not be covered in this document.


## Testing in Autoware.Auto {#how-to-write-tests-and-measure-coverage-testing}

This paragraph introduces various types of tests that are run both manually and automatically.


### Style / linter tests {#how-to-write-tests-and-measure-coverage-style-linter-tests}

Some examples of tools used for style and linting are
[cpplint](https://github.com/google/styleguide/tree/gh-pages/cpplint),
[uncrustify](https://github.com/uncrustify/uncrustify).

Tests using the tools above allow Autoware.Auto to follow C and C++ style guides which results
in uniform, easy to read code.


### Static code analysis {#how-to-write-tests-and-measure-coverage-static-code-analysis}

[Cppcheck](https://github.com/danmar/cppcheck) tool is used for applications
written in Autoware.Auto.

Static code analysis tools detect the following types of errors:

- API usage errors
- Best practice coding errors
- Buffer overflows
- Build system issues
- Class hierarchy inconsistencies
- Code maintainability issues
- Concurrent data access violations
- Control flow issues
- Cross-site request forgery (CSRF)
- Cross-site scripting (XSS)
- Deadlocks
- Error handling issues
- Hard-coded credentials
- Incorrect expression
- Insecure data handling
- Integer handling issues
- Integer overflows
- Memory—corruptions
- Memory—illegal accesses
- Null pointer dereferences
- Path manipulation
- Performance inefficiencies
- Program hangs
- Race conditions
- Resource leaks
- Rule violations
- Security best practices violations
- Security misconfigurations
- SQL injection
- Uninitialized members


### Unit tests {#how-to-write-tests-and-measure-coverage-unit-tests}

Unit testing is a software testing method by which individual units of source code
are tested to determine whether they are fit for use.

The tool used for unit testing is [gtest](https://github.com/google/googletest).
A full working example is provided below.


### Integration tests {#how-to-write-tests-and-measure-coverage-integration-tests}

In integration testing, the individual software modules are combined and tested as a group.
Integration testing occurs after unit testing.

Since integration testing greatly depends on the system architecture, Autoware.Auto provides an
integration testing tool called
[integration_tests](@ref integration-testing).

While performing integration testing the following subtypes of tests are written:

1. Fault injection testing
2. Back-to-back comparison between a model and code
3. Requirements-based testing
4. Anomaly detection during integration testing
5. Random input testing


### Memory tests {#how-to-write-tests-and-measure-coverage-memory-tests}

Memory tests allow the detection of unwanted calls to memory management APIs, such as:

- `malloc`
- `calloc`
- `realloc`
- `free`

For more details on this type of tests see the
[memory testing](https://github.com/osrf/osrf_testing_tools_cpp#memory_tools).


### Software and Hardware-In-Loop tests {#how-to-write-tests-and-measure-coverage-software-and-hardware-in-loop-tests}

With software in the loop (SIL) and hardware in the loop (HIL) testing the integration of
Autoware.Auto with real sensors and ECUs is proven, as shown in the image below.

These types of tests assure that Autoware.Auto remains compatible with sensor interfaces
and specific firmware versions, for example.

![Hardware-in-the-loop setup at Apex.AI](images/hil.jpg)


### Road tests {#how-to-write-tests-and-measure-coverage-road-tests}

Tests are written for Autoware.Auto applications which are deployed and tested on the
autonomous vehicles.

These road tests validate Autoware.Auto in a realistic autonomous vehicle product. Along with road tests
Autoware.Auto is also integration tested with the research-focused counter-part
[Autoware](https://github.com/CPFL/Autoware).

![Apex.AI's testing vehicle](images/lexus.jpg)


# Write, build, run, and analyze unit tests {#how-to-write-tests-and-measure-coverage-write-build-run-and-analyze-unit-tests}

Autoware.Auto uses the `ament_cmake` framework to write, build, and run tests. The same
framework is also used to analyze the test results.

`ament_cmake` provides several convenience functions to make it easier to write
CMake-based packages:

1. It generates a CMake configuration file for the package which allows for passing information
(e.g. about include directories and libraries) to downstream packages
    1. It makes it easy to pass along information from recursive dependencies (and takes care of
      ordering include directories)
2. It provides an easy interface to register tests and ensure that JUnit-compatible
result files are generated
    1. Currently supports a few different testing frameworks like `pytest`, `gtest`, and `gmock`
3. It allows a package to generate environment hooks to extend the environment, for example by
extending the PATH
4. It provides a CMake API to read and write `ament` resource index entries
    1. The index is created at build time and provides efficient access to information
like the available packages, messages, etc.
5. It provides an uninstall target for convenience

Below is an example of using `ament_cmake_gtest` with `colcon test`. All other tests follow
a similar pattern.

This example assumes that the package `my_cool_pkg` was generated with [autoware_create_package](https://gitlab.com/AutowareAuto/AutowareAuto/tree/master/src/tools/autoware_create_pkg).


## Writing a unit test with gtest {#how-to-write-tests-and-measure-coverage-writing-unit-test-with-gtest}

In `my_cool_pkg/test`, create gtest entrypoint `gtest_main.cpp`

```cpp
#include "gtest/gtest.h"
#include "my_cool_pkg/my_cool_pkg.hpp"
int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

and `gtest` code file `test_my_cool_pkg.cpp`:

```cpp
#include "gtest/gtest.h"
#include "my_cool_pkg/my_cool_pkg.hpp"
TEST(test_my_cool_pkg, test_hello) {
  EXPECT_EQ(my_cool_pkg::print_hello(), 0);
}
```

For more examples of `gtest` features see [the gtest](https://github.com/google/googletest).

Add an entry under `BUILD_TESTING` in the `CMakeLists.txt` to compile the test:
```
find_package(ament_cmake_gtest)
set(TEST_SOURCES test/gtest_main.cpp test/test_my_cool_pkg.cpp)
set(TEST_MY_COOL_PKG_EXE test_my_cool_pkg)
ament_add_gtest(${TEST_MY_COOL_PKG_EXE} ${TEST_SOURCES})
```

The entrypoint `main` calls all tests that are registered as `gtest` items.

To register a new `gtest` item, wrap the test code with the macro **TEST ()**. **TEST ()**
is a predefined macro that helps generate the final test code, and also registers
a `gtest` item.

`gtest/gtest.h` also contains predefined macros of gtest like `ASSERT_TRUE(condition)`,
`ASSERT_FALSE(condition)`, `ASSERT_EQ(val1,val2)`, `ASSERT_STREQ(str1,str2)`,
`EXPECT_EQ()`, etc. `ASSERT_*` will abort the test if the condition is not
satisfied while `EXPECT_*` will mark the test as failed but continue to next test
condition. More information about `gtest` can be found in
[gtest repo](https://github.com/google/googletest).

In the demo `CMakeLists.txt`, `ament_add_gtest` is a predefined macro in `ament_cmake`
that helps simplify adding `gtest` code. Details can be viewed in [ament_add_gtest.cmake](https://github.com/ament/ament_cmake/blob/master/ament_cmake_gtest/cmake/ament_add_gtest.cmake).


### Build test {#how-to-write-tests-and-measure-coverage-build-test}

All necessary test files (ELF, CTesttestfile.cmake, etc.) are compiled by default by `colcon`:

```bash
ade$ cd ~/workspace/
ade$ colcon build --merge-install --packages-select my_cool_pkg
```

Test files will be generated under `~/workspace/build/my_cool_pkg`.


### Run test {#how-to-write-tests-and-measure-coverage-run-test}

To run test on a specific package, call:

```bash
ade$ colcon test --merge-install --packages-select my_cool_pkg

Starting >>> my_cool_pkg
Finished <<< my_cool_pkg [7.80s]

Summary: 1 package finished [9.27s]
```

\note  Remove `--merge-install` if the package is built without `--merge-install`, which is
equivalent to adding `--isolated` to `ament.py build` (the legacy build tool).

The test command output contains a brief report of all the test results.

To get job-wise information of all executed tests, call:

```bash
ade$ colcon test-result --all

build/my_cool_pkg/test_results/my_cool_pkg/copyright.xunit.xml: 8 tests, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/cppcheck.xunit.xml: 6 tests, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/cpplint.xunit.xml: 6 tests, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/lint_cmake.xunit.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/my_cool_pkg_exe_integration_test.xunit.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/pclint.xunit.xml: 0 tests, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/my_cool_pkg/test_results/my_cool_pkg/uncrustify.xunit.xml: 6 tests, 0 errors, 0 failures, 0 skipped

Summary: 29 tests, 0 errors, 0 failures, 0 skipped

```

Look in the `~/workspace/log/test_<date>/<package_name>` directory for all the raw test
commands, `std_out`, and `std_err`. There's also the `~/workspace/log/latest_*/` directory
containing symbolic links to the most recent package-level build and test output.

To print the tests' details while the tests are being run, use the
`--event-handlers console_cohesion+` option to print the details directly to the console:

```bash
ade$ colcon test --merge-install --event-handlers console_cohesion+ --packages-select my_cool_pkg

...
test 1
    Start 1: test_my_cool_pkg

1: Test command: /usr/bin/python3 "-u" "~/workspace/install/share/ament_cmake_test/cmake/run_test.py" "~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml" "--package-name" "my_cool_pkg" "--output-file" "~/workspace/build/my_cool_pkg/ament_cmake_gtest/test_my_cool_pkg.txt" "--command" "~/workspace/build/my_cool_pkg/test_my_cool_pkg" "--gtest_output=xml:~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml"
1: Test timeout computed to be: 60
1: -- run_test.py: invoking following command in '~/workspace/src/my_cool_pkg':
1:  - ~/workspace/build/my_cool_pkg/test_my_cool_pkg --gtest_output=xml:~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml
1: [==========] Running 1 test from 1 test case.
1: [----------] Global test environment set-up.
1: [----------] 1 test from test_my_cool_pkg
1: [ RUN      ] test_my_cool_pkg.test_hello
1: Hello World
1: [       OK ] test_my_cool_pkg.test_hello (0 ms)
1: [----------] 1 test from test_my_cool_pkg (0 ms total)
1:
1: [----------] Global test environment tear-down
1: [==========] 1 test from 1 test case ran. (0 ms total)
1: [  PASSED  ] 1 test.
1: -- run_test.py: return code 0
1: -- run_test.py: inject classname prefix into gtest result file '~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml'
1: -- run_test.py: verify result file '~/workspace/build/my_cool_pkg/test_results/my_cool_pkg/test_my_cool_pkg.gtest.xml'
1/8 Test #1: test_my_cool_pkg ...................   Passed    0.09 sec

...

100% tests passed, 0 tests failed out of 8

Label Time Summary:
copyright      =   0.31 sec (1 test)
cppcheck       =   0.31 sec (1 test)
cpplint        =   0.38 sec (1 test)
gtest          =   0.09 sec (1 test)
integration    =   0.58 sec (1 test)
lint_cmake     =   0.31 sec (1 test)
linter         =   7.23 sec (6 tests)
pclint         =   5.57 sec (1 test)
uncrustify     =   0.35 sec (1 test)

Total Test time (real) =   7.91 sec
...
```


# Coverage  {#how-to-write-tests-and-measure-coverage-coverage}

Loosely described, a coverage metric is a measure of how much of the program code
has been exercised (covered) during testing.

In Autoware.Auto the [lcov tool] (http://ltp.sourceforge.net/documentation/technical_papers/gcov-ols2003.pdf)
(which uses `gcov` internally) is used to measure:

1. Statement coverage
2. Function coverage
3. Branch coverage

`lcov` also collects the results and generates `html` to visualize the coverage
information. In the sections below, the commands to generate coverage information for `my_cool_pkg`
are shown below.

Use the commands below to generate coverage information for `my_cool_pkg`:

```bash
# In ~/workspace/
# Build with correct flags
ade$ COVERAGE_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1"
ade$ colcon build \
          --merge-install \
          --packages-select my_cool_pkg \
          --cmake-args \
            -DCMAKE_BUILD_TYPE=Coverage \
          --ament-cmake-args \
            -DCMAKE_CXX_FLAGS="$COVERAGE_FLAGS" \
            -DCMAKE_C_FLAGS="$COVERAGE_FLAGS"


# Get a zero-coverage baseline
ade$ mkdir $(pwd)/lcov
ade$ lcov --base-directory $(pwd) --capture --initial --directory $(pwd)/build/my_cool_pkg \
-o $(pwd)/lcov/lcov.base --no-external --rc lcov_branch_coverage=1

# Run unit and integration tests
ade$ colcon test \
    --merge-install \
    --packages-select my_cool_pkg \
    --abort-on-error \
    --ctest-args -E __rmw_micro_dds_c\|pclint\|copyright\|cppcheck\|cpplint\|flake8\|lint_cmake\|pep257\|uncrustify

# Get coverage
ade$ lcov \
  --base-directory $(pwd) \
  --capture \
  --directory $(pwd)/build/my_cool_pkg \
  --output-file $(pwd)/lcov/lcov.run \
  --no-external \
  --rc lcov_branch_coverage=1

# Combine zero-coverage with coverage information.
ade$ lcov \
  -a $(pwd)/lcov/lcov.base \
  -a $(pwd)/lcov/lcov.run \
  -o $(pwd)/lcov/lcov.total \
  --rc lcov_branch_coverage=1

# Filter test, build, and install files and generate html
ade$ lcov -r $(pwd)/lcov/lcov.total \
    "*/build/*" \
    "*/install/*" \
    "CMakeCCompilerId.c" \
    "CMakeCXXCompilerId.cpp" \
    -o $(pwd)/lcov/lcov.total.filtered \
    --rc lcov_branch_coverage=1

ade$ genhtml -o $(pwd)/lcov/ $(pwd)/lcov/lcov.total.filtered -p $(pwd) --legend \
--demangle-cpp --rc genhtml_branch_coverage=1

# See coverage
ade$ chromium-browser --no-sandbox lcov/index.html
```

The resulting `lcov/index.html` will have a similar form to the following:

![Example lcov output](images/lcov_result.jpg)

In Autoware.Auto coverage metrics are run as part of the CI pipeline.
