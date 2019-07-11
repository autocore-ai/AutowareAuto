Guidelines for contributing to Autoware.Auto {#cpp-development-process}
============

# Languages

Only C++14 is allowed for production code. Python and Bash is allowed for tooling.
Deviations need to be approved by the technical leads.

# Commiting

The developer shall commit and push regularly to the server to avoid data loss.

# IDEs

While we provide an ADE volume for Atom for convenince, no specific IDE is required to develop software for Autoware.Auto.

However, we recommend using an IDE that has good support for ROS 2, such as Atom, Visual Studio Code or CLion.

## CCache

CCache should be used if no specific debugging task is done that CCache interferers
with. It is enabled by adding the following to the `.bashrc`:

```
export CC="/usr/lib/ccache/gcc"
export CXX="/usr/lib/ccache/g++"
```

Set a large cache size to make it really effective: `ccache --max-size=100G`.

The following issue needs to be considered when using CCache:
* It hides unused variables compiler warnings, which will then pop up in the CI. However,
static code analysis should also catch these locally.


# Preparations

## Creating the workspace (skip this step if you already have one)

If you plan to use the ADE (which we highly encourage you to do), you'll first need an ADE home:

```bash
$ mkdir -p ~/adehome
$ cd ~/adehome
$ touch .adehome
```

Install `ade-cli` via `pip`:

```bash
$ pip3 install -U ade-cli
```

Clone the AutowareAuto project:

```bash
$ git clone https://gitlab.com/AutowareAuto/AutowareAuto.git
```

## Entering ADE

* `cd AutowareAuto`
* `ade start --update`
* `ade enter`

## Workflow

We follow the standard fork and pull model popularized by GitHub.

All contributions start with a fork of https://gitlab.com/AutowareAuto/AutowareAuto.git into your account.

![Autoware.Auto fork](images/autoware-fork.png)

Create a branch that best represents the work you are going to start. If it is related to an existing ticket,
please prefix your branch with the number of the ticket.

Make any changes to the source code you think appropriate. We advise committing and pushing often to avoid lose of data.

The first time you push your branch, Gitlab will print a URL from where you can directly submit a merge
request (put simply, GitLab's name for what GitHub calls a pull request).

![Autoware.Auto merge request from push command line](images/autoware-merge-request.png)

A merge request can also be triggered by going to the [Autoware project page](https://gitlab.com/AutowareAuto/AutowareAuto),
where a message will pop up with a button to create new merge request.

![Autoware.Auto merge request from project page](images/autoware-merge-request-project.png)

Before a merge request can be merged, an approval from a member of the development team is required.

Additionally, our continuous integration system will run tests for any merge request and if it fails, the merge
request will be blocked from being merged.

We assign the merge request to the reviewer(s) or to the author to decide at any moment who should be taking the next action items.

When you submit a merge request for review, you should first assign it to yourself, until you think it's ready for review.

If you don't know who should be the reviewers, feel free to assign your merge request to the project leads (\@esteve and \@gbiggs),
and they will assign it to the right person.

## Initial Build

* `colcon build`

## Optional: Creating a new package

See https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/#create-your-own-package

## Writing Code

Write your code. It is recommended to build you code after every few lines using `colcon build` to validate that your code compiles.

## Optional: Running Executables

After adding an executable you can run it by invoking `ros2 run package_name executable_name`.

## Writing Integration Tests

[launch_testing](https://github.com/ros2/launch/tree/master/launch_testing#launch_testing), introduced in ROS 2 Dashing, is the recommended mechanism
for writing integration tests.

Any package that has an executable will be required to have an accompanying integration test. The purpose of the integration test is to start the executable and ensure that it
processes any input data correctly and produces outputs according to its specification.

For more information on integration testing, [see this article on integration testing fundamentals](http://softwaretestingfundamentals.com/integration-testing/).

## Checking Quality of the Code

This section discusses our requirements for validating the quality of submitted code. A failed quality check will require that the code needs to be changed or more unit tests need to be added. Therefore one might need to go back to the previous section.

# Coverage

For [code coverage](https://en.wikipedia.org/wiki/Code_coverage), we use the popular [GNU tool `lcov`](http://ltp.sourceforge.net/coverage/lcov.php) for generating statistics about line coverage.

For every merge request, we run our testsuite and report the percentage of lines covered. We aim at a 100% line coverage and continuously improve our testsuite to achieve that number. In particular, we do not accept changes that reduce the coverage value. If a merge request has a lower line coverage than `master`, we will request the contributor to add more unit tests.

The following commands, taken from our automated CI pipeline, are _one example_ of how to generate a coverage report for Autoware.Auto.
You can execute them in your Autoware.Auto workspace and confirm that the coverage value is sufficient before submitting your merge request.
Note that this is _not required_, as the CI pipeline will produce a coverage report automatically for your merge request.

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --ament-cmake-args -DCMAKE_CXX_FLAGS="$COVERAGE_FLAGS" -DCMAKE_C_FLAGS="$COVERAGE_FLAGS"
lcov --config-file .lcovrc --base-directory "$PWD" --capture --directory build -o lcov.base --initial
colcon test --abort-on-error
lcov --config-file .lcovrc --base-directory "$PWD" --capture --directory build -o lcov.test
lcov --config-file .lcovrc -a lcov.base -a lcov.test -o lcov.total
lcov --config-file .lcovrc -r lcov.total "*/AutowareAuto/install/*" "*/CMakeCCompilerId.c" "*/CMakeCXXCompilerId.cpp" "*_msgs/*" -o lcov.total.filtered
genhtml --config-file .lcovrc -p "$PWD" --legend --demangle-cpp lcov.total.filtered -o coverage
```

# Static Code Analysis

Static code analysis should be run in the described order, as the analyses are ordered from a semantic focus to a syntactic focus.

## Linting and formatting code

`ament` comes with extensions for linting and formatting soure code via `cpplint` and `uncrustify`.

In order to check that a C/C++ file (header or implementation) does not contain serious static errors, you can invoke `ament_cpplint filename`.

Additionally, to ensure that source files are formatted according to the ROS 2 style guidelines, type `ament_uncrustify filename --reformat`.

## Documentation

To check that the code is properly documented and all documents are correctly linked run  `AutowareAuto/docs/.doxygen/build.py`.

The generated documentation can be found in `docs/_build/html/index.html`.

## Finalizing the Code

# Running all Tests of a Package

To run all tests of the package you are working on, run `colcon test --packages-select packagen_name`. Afterwards, you will need to invoke `colcon test-result --verbose` to see more details of any tests that failed.

## Cleaning up Commits

Commits should be explicit about the goal of the contribution.

Before merging a merge request, all commits will be squashed to maintain a clean commit log.

There are several options to cleanup your commits using rebasing:

* Use the terminal: `git rebase -i master`.

To push your chances run `git push -fpush` which allows history changes introduced by rebasing.
