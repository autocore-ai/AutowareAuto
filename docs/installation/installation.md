Installation {#installation}
============

[TOC]

# Goals {#installation-and-development-goals}


This article demonstrates how to use the Agile Development Environment (ADE) to develop Autoware.Auto applications.


# Install ADE {#installation-and-development-install-ade}

[ADE](https://ade-cli.readthedocs.io/en/latest/) is a modular Docker-based tool to ensure that all developers in a project have a common, consistent development environment.

Follow the [install](https://ade-cli.readthedocs.io/en/latest/install.html) instructions, which are reproduced here for convenience:

1. Verify that the requirements [listed here](https://ade-cli.readthedocs.io/en/latest/install.html#requirements) are fulfilled
1. Download the statically-linked binary from the [Releases](https://gitlab.com/ApexAI/ade-cli/-/releases) page of the `ade-cli` project
1. Name the binary `ade` and install it in your `PATH` (on Ubuntu, `/usr/local/bin` is recommended)
1. Make the binary executable: `chmod +x ade`
1. Check that it is installed:

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
$ git clone --recurse-submodules git@gitlab.com:autowarefoundation/autoware.auto/AutowareAuto.git
```


# How to build {#installation-and-development-how-to-build}

```
$ ade start --update --enter 
ade$ cd AutowareAuto
ade$ colcon build
ade$ colcon test
ade$ colcon test-result
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

# Troubleshooting

## General Troubleshooting

Most issues with building Autoware.AUTO are caused by out-of-date software or old build files.
To update `ade` and the Docker containers it manages as well as clear old builds, run the following in your `adehome/AutowareAuto` folder:

```console
$ ade stop
$ sudo ade update-cli
$ ade start --update --enter
ade$ cd AutowareAuto
ade$ rm -rf build/ install/ log/
```

If you are still having trouble after these commands have been run, please post a request for help on [ROS Answers](https://answers.ros.org/questions/ask/?tags=autoware).

## Troubleshooting Specific Errors

### Error

When starting `ade` with GPU support enabled for Nvidia graphics, you may sometimes receive the following error:

```console
docker: Error response from daemon: OCI runtime create failed: container_linux.go:349: starting container process caused "process_linux.go:449: container init caused \"process_linux.go:432: running prestart hook 0 caused \\\"error running hook: exit status 1, stdout: , stderr: nvidia-container-cli: initialization error: cuda error: forward compatibility was attempted on non supported hw\\\\n\\\"\"": unknown.
ERROR: Command return non-zero exit code (see above): 125
```

This usually indicates that a new Nvidia graphics driver has been installed (usually via `apt`) but the system has not yet been restarted.

### Solution

Restart your system after installing the new NVIDIA driver.
