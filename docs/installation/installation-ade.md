Installation with ADE {#installation-ade}
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

```{bash}
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
$ mkdir -p ~/adehome
$ cd ~/adehome
$ touch .adehome
```

For ADE to function, it must be properly configured. Autoware.Auto provides
an [.aderc](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/blob/master/.aderc) file
which is expected to exist in the current working
directory, or in any parent directory. Additionally, default configuration values can be
overridden by setting environment variables. See the `ade --help` output for more information about
using environment variables to define the configuration.

```
$ cd ~/adehome
$ git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
```

## Sharing files between your host and ADE
You might want to share files such as dotfiles or handy programs from your host machine with ADE. If you only have
a single `adehome` directory, there is a way to do that without duplicating them: move them inside the `adehome` directory,
then symlink them to their regular location. For instance, you could move `~/.bashrc` to `~/adehome/.bashrc` and make
`~/.bashrc` be a symlink to that. It will then appear as `~/.bashrc` to the host machine and to ADE. You
can also put handy programs in `~/.local/bin` and move the entire `~/.local` directory. The opposite direction
will not work, files in a Docker container can not be symlinks to the outside.


# Entering your development environment
```
$ cd AutowareAuto
```

To start the default environment:

```
$ ade start --update --enter
```

There are several preconfigured environments to choose from by specifying an ADE rc file. To see
what is available, run

```
ls -l .aderc*
```

Choose one, then launch with:

```
ade --rc .aderc-amd64-foxy  start --update --enter
```

Now you should have a terminal inside ADE, ready for @ref usage, or for developing Autoware.Auto.


# Cleanup {#installation-and-development-cleanup}

ADE uses Docker, and over time unused images, containers, and volumes begin to clutter the hard
drive. Follow the steps below to clean the Docker filesytem of stale images.


## Start relevant Docker resources {#installation-and-development-start-relevant-docker-resources}

First, verify that ADE is running:

```
$ cd ~/adehome/AutowareAuto
$ ade start
```

If ADE is used for more than one project, verify all ADE instances are running; the same rule
applies for any other non-ADE Docker containers that should be preserved.

\note
Docker resources that are not started/running **will be removed**!


## Docker disk usage {#installation-and-development-docker-disk-usage}

To assess the disk usage situation, run the following command:

```
$ docker system df
TYPE                TOTAL               ACTIVE              SIZE                RECLAIMABLE
Images              13                  11                  14.03GB             916.9MB (6%)
Containers          11                  0                   2.311MB             2.311MB (100%)
Local Volumes       17                  15                  5.411GB             17.8MB (0%)
Build Cache         0                   0                   0B                  0B
```


## Remove unused docker items {#installation-and-development-remove-unused-docker-items}

Use `docker system prune` to remove any Docker items not used for currently running containers:

```
$ docker system prune -a --volumes
```

# Troubleshooting {#ade-troubleshooting}
Here are solutions for a few specific errors:

## Error - "forward compatibility was attempted on non supported hw" when starting ADE

When starting `ade` with GPU support enabled for NVIDIA graphics, you may sometimes receive the following error:

```
docker: Error response from daemon: OCI runtime create failed: container_linux.go:349: starting container process caused "process_linux.go:449: container init caused \"process_linux.go:432: running prestart hook 0 caused \\\"error running hook: exit status 1, stdout: , stderr: nvidia-container-cli: initialization error: cuda error: forward compatibility was attempted on non supported hw\\\\n\\\"\"": unknown.
ERROR: Command return non-zero exit code (see above): 125
```

This usually indicates that a new NVIDIA graphics driver has been installed (usually via `apt`) but the system has not yet been restarted. A similar message may appear if the graphics driver is not available, for example because of resuming after suspend.

### Solution

Restart your system after installing the new NVIDIA driver.

## Error - "Unable to create the rendering window after 100 tries" when launching GUI application

If you have an NVIDIA GPU and are using the proprietary NVIDIA GPU driver, you may encounter this error when using the default `.aderc` or `.aderc-arm64` files.
This is due to a decision that was made regarding support for users with and without NVIDIA GPUs and those with and without the proprietary NVIDIA driver.
For more information you can review the discussion that lead to this decision in [this issue](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/502).

To resolve this issue, simply remove the line `export ADE_DISABLE_NVIDIA_DOCKER=true` from the `.aderc` file that you are using and restart `ade` with:

```
ade$ exit
$ ade stop
$ ade start --update --enter
```
