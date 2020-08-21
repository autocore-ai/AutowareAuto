Troubleshooting {#troubleshooting}
===============

[TOC]

# Troubleshooting Guidelines {#troubleshooting-guidelines}

When troubleshooting problems with Autoware.Auto, the following should be the general workflow:

1. Review this troubleshooting guide for possible solutions.
2. Post a question to [ROS Answers](https://answers.ros.org/questions/ask/?tags=autoware) with the `autoware` tag.
  - If your post on ROS Answers confirms that your issue exists for others and is not specific to your configuration, then it can be considered a "Confirmed Bug."
3. If your issue is a "Confirmed Bug," [create a new issue](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/new) on the Autoware.Auto Gitlab site.
  - Remember to select an issue template and fill out all fields to help us in identifying a fix.
4. If you have a solution to the "Confirmed Bug," please feel free to create a Merge Request tied to the above issue.
  - We use issues for tracking development progress and planning.
    We will not accept a merge request which is not tied to an issue.

# General Troubleshooting Steps {#general-troubleshooting-steps}

Most issues with building Autoware.Auto are caused by out-of-date software or old build files.
To update `ade` and the Docker containers it manages as well as clear old builds, run the following in your `adehome/AutowareAuto` folder:

```console
$ ade stop
$ sudo ade update-cli
$ ade start --update --enter
ade$ cd AutowareAuto
ade$ rm -rf build/ install/ log/
ade$ git pull
```

If you are using Autoware.Auto outside of `ade`, try updating your system and running the following in your AutowareAuto folder and re-building:

```console
$ rm -rf build/ install/ log/
$ git pull
```

If you are still having trouble after these commands have been run, please see the @ref troubleshooting-guidelines above or the @ref solutions-for-specific-errors below.

# Solutions for Specific Errors {#solutions-for-specific-errors}

## Error - "forward compatibility was attempted on non supported hw" when starting ADE

When starting `ade` with GPU support enabled for NVIDIA graphics, you may sometimes receive the following error:

```console
docker: Error response from daemon: OCI runtime create failed: container_linux.go:349: starting container process caused "process_linux.go:449: container init caused \"process_linux.go:432: running prestart hook 0 caused \\\"error running hook: exit status 1, stdout: , stderr: nvidia-container-cli: initialization error: cuda error: forward compatibility was attempted on non supported hw\\\\n\\\"\"": unknown.
ERROR: Command return non-zero exit code (see above): 125
```

This usually indicates that a new NVIDIA graphics driver has been installed (usually via `apt`) but the system has not yet been restarted.

### Solution

Restart your system after installing the new NVIDIA driver.

## Error - "Unable to create the rendering window after 100 tries" when launching GUI application

If you have an NVIDIA GPU and are using the proprietary NVIDIA GPU driver, you may encounter this error when using the default `.aderc` or `.aderc-arm64` files.
This is due to a decision that was made regarding support for users with and without NVIDIA GPUs and those with and without the proprietary NVIDIA driver.
For more information you can review the discussion that lead to this decision in [this issue](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/502).

To resolve this issue, simply remove the line `export ADE_DISABLE_NVIDIA_DOCKER=true` from the `.aderc` file that you are using and restart `ade` with:

```console
ade$ exit
$ ade stop
$ ade start --update --enter
```
