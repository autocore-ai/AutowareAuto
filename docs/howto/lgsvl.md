LGSVL simulator {#lgsvl}
========

[TOC]

# LGSVL simulator: running the LGSVL simulator alongside Autoware.Auto

The following guide assumes that the LGSVL simulator will be run from inside an ADE container.

## Requirements

- ADE 4.1.0 or later. Follow the instructions at https://ade-cli.readthedocs.io/en/latest/install.html to install it
- An ADE volume is provided that contains the LGSVL in `registry.gitlab.com/apexai/ade-lgsvl:2019.11`

## Instructions

Run ADE as described in the [installation section](installation-and-development.html#installation-and-development-install-ade):

* `cd AutowareAuto`
* `ade start --update`
* `ade enter`
