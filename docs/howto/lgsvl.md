LGSVL simulator {#lgsvl}
========

[TOC]

# LGSVL simulator: running the LGSVL simulator alongside Autoware.Auto

The following guide assumes that the LGSVL simulator will be run from inside an ADE container.

## Requirements

- ADE 4.1.0 or later. Follow the instructions at https://ade-cli.readthedocs.io/en/latest/install.html to install it
- An ADE volume is provided that contains the LGSVL in `registry.gitlab.com/apexai/ade-lgsvl:2019.12`

## Instructions

Install ADE as described in the [installation section](installation-and-development.html#installation-and-development-install-ade):

Start ADE with the LGSVL volume:

* `export ADE_DOCKER_RUN_ARGS="--cap-add=SYS_PTRACE"`
* `export ADE_GITLAB=gitlab.com`
* `export ADE_REGISTRY=registry.gitlab.com`
* `export ADE_IMAGES="
  registry.gitlab.com/autowarefoundation/autoware.auto/autowareauto/ade:master
  registry.gitlab.com/apexai/ade-atom:latest
  registry.gitlab.com/autowarefoundation/autoware.auto/autowareauto:master
  registry.gitlab.com/apexai/ade-lgsvl:2019.12
 "`
* `cd adehome/AutowareAuto`
* `ade start --update -- --net=host`
* `ade enter`

Start the ROS 2 web bridge from inside ADE:

* `rosbridge &`

Start the LGSVL simulator:

* `/opt/lgsvl/simulator`

Now start your favorite browser and go to http://localhost:8080 where simulations can be configured.

Follow the instructions in https://www.lgsvlsimulator.com/docs/autoware-auto-instructions/#run-simulator-alongside-autowareauto to configure the Lexus model to use the ROS 2 bridge.
