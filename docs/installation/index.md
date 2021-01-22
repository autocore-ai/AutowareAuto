Installation {#installation}
============================

The recommended method for installation is through the use of [ADE](https://ade-cli.readthedocs.io/en/latest/),
a Docker-based tool to ensure that all developers in a project have a common, consistent development
environment. It comes with a pre-built version of Autoware.Auto, so that you will not need to compile it yourself
if you do not want to.

Autoware.auto can also be built without the use of [ADE](https://ade-cli.readthedocs.io/en/latest/)
for cases where a more granular control of the installation environment is needed.

- @subpage installation-ade
- @subpage installation-no-ade

Another prerequisite for running the full software stack is the LGSVL simulator:

- @subpage lgsvl