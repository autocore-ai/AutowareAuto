Installation and development setup {#installation-and-development}
============

[TOC]

# Goals {#installation-and-development-1}


In this article we will demonstrate how to use Agile Development Environment
to develop AutowareAuto applications.

# Install ADE {#install-ADE}

ADE is published on PyPI. It needs Python >= 3.5.2 and pip. All other
dependencies will be fetched by pip.

```
$ sudo apt-get install python3-pip
$ pip3 install ade-cli
```


# Setup ADE home and project checkout {#setup-ADE-home-and-project-checkout}

ADE needs a directory on the host which will be mounted as the user's
home directory within the container. It will be populated with
dotfiles and must be different than the user's home directory
*outside* the container. In case you use ADE for multiple projects it
is recommended to use dedicated adehome directories per project.

ADE will look for a directory containing a file named ``.adehome``
starting with the current working directory and continuing with the
parent directories to identify the ADE home directory to be mounted.

```
$ mkdir adehome
$ cd adehome
$ touch .adehome
```

For ade to function it needs to be configured. Autoware.Auto provides
a [.aderc](https://gitlab.com/AutowareAuto/AutowareAuto/blob/master/.aderc)
which is looked for in the current working
directory and any parent directories. Additionally, values can be
overridden by setting environment variables.

```
$ cd adehome
$ git clone git@gitlab.com:AutowareAuto/AutowareAuto.git
$ cd AutowareAuto
$ ade start
$ ade enter
```


# How to build {#how-to-build}

```
$ ade enter
ade$ cd AutowareAuto
ade$ colcon build
ade$ colcon test
ade$ colcon test-result
```


# How to use Atom for development {#how-to-use-Atom-for-development}

The Autoware.Auto ADE image ships with the [Atom](https://atom.io/) text editor
and automatically [installs](https://gitlab.com/AutowareAuto/AutowareAuto/blob/master/tools/ade_image/atom-install-our-plugins)
some useful Atom packages. Be sure to checkout its Welcome Guide and make yourself
familiar with its features and keyboard shortcuts.


## Prepare your workspace {#prepare-your-workspace}

To use all Atom features, like gdb debugging, clang autocompletion, and ctags
supported code navigation you have to prepare your workspace:

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


## Navigate through the code {#navigate-through-the-code}

In order for Atom to use the global `tags` file open the AutowareAuto folder as
a *Project Folder* (File -> Add Project Folder...).

You should add packages you are working on as additional project folders, but
always keep AutowareAuto added to give Atom an overview of the full project.

The two main shortcuts you are going to use are `F6` to go to the declaration
of a symbol and `CTRL-SPACE` to go to its implementation and `CTRL-SHIFT-SPACE`
to quickly return.


## Auto-format your code {#auto-format-your-code}

Atom can auto-format your code to adhere to the ROS2 style rules using the
configuration file from ament_uncrustify. The shortcut `CTRL-ALT-U` will
automatically reformat the currently open file.


## Build code from Atom {#build-code-from-Atom}

The installed packages include *build-colcon*, a colcon specific provider for
the Atom *build*  package. To take advantage of build-colcon, you have to open a
ROS2 package as a *Project Folder* (File -> Add Project Folder...). All the
functions and shortcuts provided by [build](https://atom.io/packages/build)
should be available out of the box. Pressing `F9` builds the current project.


## Run build packages from Atom {#run-build-packages-from-Atom}

To test code directly from Atom press ``CTRL-` `` to open the built-in terminal.
Here you can run a built binary:

```bash
ade$ ~/AutowareAuto/build/demo_nodes_cpp/talker
```


## Debug a binary with gdb {#debug-a-binary-with-gdb}

To debug a binary during runtime you can use the integrated gdb interface to
introspect and step through the code.

### Example {#example}

To debug `listener` from the `demo_nodes_cpp` package add
`AutowareAuto/tools/demo_nodes_cpp` as a project folder.

Open `src/topics/listener.cpp` and add a breakpoint on line 53 by clicking the
blank space next to the line number.

Click `Launch debugger...` in the sidebar on the right. Make sure the `Native -
GDB` tab is selected and enter `~/AutowareAuto/build/demo_nodes_cpp/listener`
as the program and `~` as the current working directory. The remaining two
entries can be left blank. Clicking `Launch` will start the listener binary and
pause at the breakpoint.


# Cleanup {#cleanup}

ADE uses docker and over time unused images, containers and volumes
will clutter your hard drive.


## Start up everything docker you want to keep {#Start-up-everything-docker-you-want-to-keep}

Let's first make sure that ADE is running:

```console
$ cd adehome/AutowareAuto
$ ade start
```

In case you use ade for more than one project make sure all of them
are running, same for any other docker containers you want to keep.


## Docker disk usage {#Docker-disk-usage}

To assess the situation:

```console
$ docker system df
TYPE                TOTAL               ACTIVE              SIZE                RECLAIMABLE
Images              13                  11                  14.03GB             916.9MB (6%)
Containers          11                  0                   2.311MB             2.311MB (100%)
Local Volumes       17                  15                  5.411GB             17.8MB (0%)
Build Cache         0                   0                   0B                  0B
```


## Remove unused docker items {#Remove-unused-docker-items}

```console
$ docker system prune -a --volumes
```
