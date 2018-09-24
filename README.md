# Autoware.Auto

Autoware.Auto uses the Agile Development Environment (ADE).


## Documentation

The latest documentation corrsponding to the ``master`` branch can be found here:
https://autowareauto.gitlab.io/AutowareAuto/


## Install ADE

ADE is published on PyPI. It needs Python >= 3.5.2 and pip. All other
dependencies will be fetched by pip.

```
sudo apt-get install python3-pip
pip3 install ade-cli
```


## Setup ADE home and project checkout

ADE needs a directory on the host which will be mounted as the user's
home directory within the container. It will be populated with
dotfiles and must be different than the user's home directory
*outside* the container. In case you use ADE for multiple projects it
is recommended to use dedicated adehome directories per project.

ADE will look for a directory containing a file named ``.adehome``
starting with the current working directory and continuing with the
parent directories to identify the ADE home directory to be mounted.

```
mkdir adehome
cd adehome
touch .adehome
```

For ade to function it needs to be configured. Autoware.Auto provides
a [.aderc](./.aderc) which is looked for in the current working
directory and any parent directories. Additionally, values can be
overridden by setting environment variables.

```
cd adehome
git clone git@gitlab.com:AutowareAuto/AutowareAuto.git
cd AutowareAuto
ade start
ade enter
```


## How to build

```
ade enter
cd AutowareAuto
colcon build
colcon test
colcon test-result
```


## How to use Atom for development

The Autoware.Auto ADE image ships with the [Atom](https://atom.io/) text editor
and automatically [installs](tools/ade_image/atom-install-our-plugins) some
useful Atom packages.

The installed packages include *build-colcon*, a colcon specific provider for
the *build*  package. To take advantage of build-colcon, you have to open a
ROS2 package as a *Project Folder* (File -> Add Project Folder...). All the
functions and shortcuts provided by [build](https://atom.io/packages/build)
should be available out of the box.


## Cleanup

ADE uses docker and over time unused images, containers and volumes
will clutter your hard drive.


### Start up everything docker you want to keep

Let's first make sure that ADE is running:

```console
cd adehome/AutowareAuto
ade start
```

In case you use ade for more than one project make sure all of them
are running, same for any other docker containers you want to keep.


### Docker disk usage

To assess the situation:

```console
$ docker system df
TYPE                TOTAL               ACTIVE              SIZE                RECLAIMABLE
Images              13                  11                  14.03GB             916.9MB (6%)
Containers          11                  0                   2.311MB             2.311MB (100%)
Local Volumes       17                  15                  5.411GB             17.8MB (0%)
Build Cache         0                   0                   0B                  0B
```


### Remove unused docker items

```
docker system prune -a --volumes
```
