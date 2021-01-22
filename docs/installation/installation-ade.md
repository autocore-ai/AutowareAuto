Installation with ADE{#installation-ade}
=====================

[TOC]

# Goals {#installation-and-development-goals}


This article demonstrates how to use the Agile Development Environment (ADE) to develop Autoware.Auto applications.


# Install ADE {#installation-and-development-install-ade}

[ADE](https://ade-cli.readthedocs.io/en/latest/) is a modular Docker-based tool to ensure that all developers in a project have a common, consistent development environment.

Follow the [install](https://ade-cli.readthedocs.io/en/latest/install.html) instructions, which are reproduced here for convenience:

1. Verify that the requirements [listed here](https://ade-cli.readthedocs.io/en/latest/install.html#requirements) are fulfilled
2. Download the statically-linked binary from the [Releases](https://gitlab.com/ApexAI/ade-cli/-/releases) page of the `ade-cli` project
3. Name the binary `ade` and install it in your `PATH` (on Ubuntu, `/usr/local/bin` is recommended)
4. Make the binary executable: `chmod +x ade`
5. Check that it is installed:

```bash
$ which ade
/path/to/ade
$ ade --version
<version>
```

# Setup ADE home and project checkout {#installation-and-development-setup-ade-home-and-project-checkout}

ADE needs a directory on the host machine which is mounted as the user's
home directory within the container. The directory is populated with
dotfiles, and must be different than the user's home directory
*outside* of the container. In the event ADE is used for multiple, projects it
is recommended to use dedicated `adehome` directories for each project.

ADE looks for a directory containing a file named `.adehome`
starting with the current working directory and continuing with the
parent directories to identify the ADE home directory to be mounted.

```
$ mkdir adehome
$ cd adehome
$ touch .adehome
```

For ADE to function, it must be properly configured. Autoware.Auto provides
an [.aderc](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/blob/master/.aderc) file
which is expected to exist in the current working
directory, or in any parent directory. Additionally, default configuration values can be
overridden by setting environment variables. See the `ade --help` output for more information about
using environment variables to define the configuration.

```
$ cd adehome
$ git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
```


# How to build {#installation-and-development-how-to-build}

```
$ ade start --update --enter
ade$ cd AutowareAuto
ade$ vcs import < autoware.auto.$ROS_DISTRO.repos
ade$ colcon build
ade$ colcon test
ade$ colcon test-result
```

# Choosing a DDS Vendor

Choosing a DDS vendor is usually as simple as changing the `RMW_IMPLEMENTATION` environment variable.
This can either be done by changing the value that is added to the `.aderc` file in the `ADE_DOCKER_RUN_ARGS` or by overriding it manually using the commands below.
For more information about why you would want to use a different DDS vendor and which ones are available, see [this ROS Index article](https://index.ros.org/doc/ros2/Concepts/DDS-and-ROS-middleware-implementations/).
For more information about working with multiple middleware (DDS) implementations, see [this ROS Index article](https://index.ros.org/doc/ros2/Tutorials/Working-with-multiple-RMW-implementations/).

## For Cyclone DDS (the default in `ade`):

```
ade$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## For FastRTPS (now FastDDS - the default in ROS Dashing):
```
ade$ export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## For Connext (not installed to `ade` by default):
```
ade$ sudo apt-get install rti-connext-dds-5.3.1 ros-dashing-rmw-connext-cpp
ade$ export RMW_IMPLEMENTATION=rmw_connext_cpp
```

## For GurumDDS (not installed to `ade` by default):
```
ade$ sudo apt-get install gurumdds-2.6 ros-dashing-rmw-gurumdds-cpp
ade$ export RMW_IMPLEMENTATION=rmw_gurumdds_cpp
```

# Cleanup {#installation-and-development-cleanup}

ADE uses Docker, and over time unused images, containers, and volumes begin to clutter the hard
drive. Follow the steps below to clean the Docker filesytem of stale images.


## Start relevant Docker resources {#installation-and-development-start-relevant-docker-resources}

First, verify that ADE is running:

```bash
$ cd adehome/AutowareAuto
$ ade start
```

If ADE is used for more than one project, verify all ADE instances are running; the same rule
applies for any other non-ADE Docker containers that should be preserved.

\note
Docker resources that are not started/running **will be removed**!


## Docker disk usage {#installation-and-development-docker-disk-usage}

To assess the disk usage situation, run the following command:

```console
$ docker system df
TYPE                TOTAL               ACTIVE              SIZE                RECLAIMABLE
Images              13                  11                  14.03GB             916.9MB (6%)
Containers          11                  0                   2.311MB             2.311MB (100%)
Local Volumes       17                  15                  5.411GB             17.8MB (0%)
Build Cache         0                   0                   0B                  0B
```


## Remove unused docker items {#installation-and-development-remove-unused-docker-items}

Use `docker system prune` to remove unused Docker items:

```console
$ docker system prune -a --volumes
```
