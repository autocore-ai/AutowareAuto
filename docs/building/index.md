Building {#building}
========

# Prerequisites
You need to be inside an ADE container, or have installed the dependencies manually. See @ref installation.

If you haven't done so already, get the source code with 

```bash
$ git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
$ cd AutowareAuto
$ vcs import < autoware.auto.$ROS_DISTRO.repos
```

Optionally, you can choose a DDS implementation other than the default Cyclone DDS: @subpage choosing-a-dds-vendor


# How to build the code {#installation-and-development-how-to-build}
To build all packages in Autoware.Auto, navigate into the AutowareAuto directory and run

```bash
ade$ colcon build
```

It's important that you always run `colcon build` from the repository root. If everything went well, you should _not_ see "failed" on your screen, although "packages had stderr output" is okay.

By default, this produces a maximally optimized build in order to run the stack as efficiently as possible. For debugging symbols and/or reduced compile times, you can add `--cmake-args -DCMAKE_BUILD_TYPE="Debug"` to the command line.

To verify that everything works as expected, see if all tests pass:

```bash
ade$ colcon test
ade$ colcon test-result --verbose
```
The first command will run the tests attached to the packages in your workspace.
The second command gives you detailed output from the tests on which ones passed and which failed.

## Advanced options
ROS 2 uses the `colcon` build system. For more information and details about options and flags, take a look at
```bash
colcon build --help
```
and see [the colcon documentation](https://colcon.readthedocs.io/en/released/user/quick-start.html). In the following, a few of the most useful options are listed.
Note that `colcon` options are spelled with an underscore instead of a dash – this is a common cause of typos.


### Selecting packages to build
To just build a single package:

```bash
colcon build --packages-select <package_name>
```

Note that this does not automatically also build or rebuild its dependencies recursively. To do that:

```bash
colcon build --packages-up-to <package_name>
```

These options are also accepted by `colcon test`.

To add a compiler flag to all packages, e.g. for enabling the undefined behavior sanitizer:
```bash
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=undefined"
```

### Cleaning the build output
`colcon` isn't very good at being stateless, so when you build, make changes, and build again, you can sometimes end up with a different result than when you build from scratch. To make sure you're getting a fresh build of a package, just do

```bash
rm -rf {build,install}/my_package
```

to remove all build artifacts associated with that package. Alternatively, if you don't want to delete the old binaries, you can specify custom build and install directories:

```bash
colcon build --build-base build_mybranch --install-base install_mybranch
```

### Seeing compiler commands
To see the compiler and linker invocations for a package, use 
```bash
`VERBOSE=1 colcon build --packages-up-to <package_name> --event-handlers console_direct+
```


## Starting from a clean slate

Most issues with building Autoware.Auto are caused by out-of-date software or old build files.
To update `ade` and the Docker containers it manages as well as clear old builds, run the following in your `adehome/AutowareAuto` folder:

```bash
$ ade stop
$ sudo ade update-cli
$ ade start --update --enter
ade$ cd AutowareAuto
ade$ rm -rf build/ install/ log/ src/external/
ade$ git pull
ade$ vcs import < autoware.auto.$ROS_DISTRO.repos
```

If you are using Autoware.Auto outside of `ade`, try updating your system and running the following in your AutowareAuto folder and re-building:

```bash
$ rm -rf build/ install/ log/ src/external/
$ git pull
$ vcs import < autoware.auto.$ROS_DISTRO.repos
```

If you are still having trouble after these commands have been run, please see the @ref support-guidelines for where to ask questions.
