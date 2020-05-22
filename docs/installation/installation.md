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


# How to use Atom for development {#installation-and-development-how-to-use-atom-for-development}

The Autoware.Auto ADE image ships with the [Atom](https://atom.io/) text editor,
and automatically
[installs](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/blob/master/tools/ade_image/atom-install-our-plugins)
some useful Atom packages. Be sure to checkout the Atom *Welcome Guide* and familiarize with the
features and keyboard shortcuts.


## Prepare the workspace {#installation-and-development-prepare-the-workspace}

To be able to use all Atom features, like `gdb` debugging, clang autocompletion, and ctags for code
navigation, prepare the workspace:

```bash
# cleanup workspace
ade$ cd ~/AutowareAuto
ade$ rm -rf .clang_complete build install log

# enable compiler wrapper script, needed for clang autocomplete
ade$ export CC=$PWD/tools/clang_complete/cc
ade$ export CXX=$PWD/tools/clang_complete/g++

# generate tags file for code navigation
# be sure to repeat this when it gets out of date
ade$ ctags -R .

# build workspace with debugging enabled, building from
ade$ colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Debug'
```


## Navigate through the code {#installation-and-development-navigate-through-the-code}

\note
The ctags file must be created before Atom can provide code navigation support;
[prepare the workspace](@ref installation-and-development-prepare-the-workspace) before following
the steps in this section.

In order for Atom to use the global `tags` file, open the `AutowareAuto` folder as
a *Project Folder* (File -> Add Project Folder...).

Add other packages that are being developed as additional project folders, but
always keep `AutowareAuto` added to give Atom an overview of the full project.

The two main shortcuts that are used for code naviation are `F6` to go to the declaration
of a symbol, `CTRL-SPACE` to go to the implementation, and `CTRL-SHIFT-SPACE` to quickly return.


## Auto-formatting code {#installation-and-development-auto-formatting-code}

Atom can auto-format the code to adhere to the ROS 2 style rules using the
configuration file from `ament_uncrustify`. The shortcut `CTRL-ALT-U` automatically reformats the
currently open file.


## Build code from Atom {#installation-and-development-build-code-from-atom}

The installed packages include `build-colcon`, a colcon specific provider for
the Atom *build*  package. To take advantage of `build-colcon`, open a
ROS 2 package as a *Project Folder* (File -> Add Project Folder...).

The functions and shortcuts provided by [build](https://atom.io/packages/build) are available out of
the box. Pressing `F9` builds the current project.


## Run packages from Atom {#installation-and-development-run-packages-from-atom}

To test code directly from Atom press '`CTRL`' + '`~`' to open the built-in terminal.
Run a built binary from the terminal, for example:

```bash
ade$ ~/AutowareAuto/build/PACKAGE_NAME/BINARY_NAME
```


## Debug a binary with GDB {#installation-and-development-debug-a-binary-with-gdb}

To debug a binary during runtime, use the integrated `gdb` interface to introspect and step through
the code.

A simple debug process might look like:

-# Open a source file for the binary
-# Add a breakpoint by clicking the blank space next to the line number
-# Click `Launch debugger...` in the sidebar on the right side of Atom
-# Go to the `Native - GDB` tab
-# Enter the binary path as the program path
-# Enter `~` as the current working directory
-# The remaining two entries can be left blank
-# Click `Launch` to start the binary and pause at the breakpoint

\note
Applications that execute and exit quickly are not seen by the debugger **unless a breakpoint is
set**.

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

Use `docker system prune` to remove unused Docker items:

```console
$ docker system prune -a --volumes
```
