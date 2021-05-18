YOLOv2 Tiny Example Pipeline {#tvm-utility-yolo-v2-tiny-tests}
===========

This is an example implementation of an inference pipeline using the pipeline
framework. This example pipeline executes the
[YOLO V2 Tiny](https://pjreddie.com/darknet/yolov2/) model and decodes its
output.

# Compiling the Example

1. Download an example image to be used as test input. this image needs to be
   saved in the `artifacts/yolo_v2_tiny/` folder

```sh
$ curl https://raw.githubusercontent.com/pjreddie/darknet/master/data/dog.jpg \
  > artifacts/yolo_v2_tiny/test_image_0.jpg
```

2. If running under `ade`, start the Autoware docker environment (see @ref installation-ade).

3. Build and test (see @ref building) with the `DOWNLOAD_ARTIFACTS` flag set.

```sh
$ colcon build --packages-up-to tvm_utility --cmake-args -DDOWNLOAD_ARTIFACTS=ON
$ colcon test --packages-select tvm_utility
```

# GPU backend

Vulkan is supported by default by the tvm_vendor package. It can be selected by setting the
`tvm_utility_BACKEND` variable:
```sh
$ colcon build --packages-up-to tvm_utility --cmake-args -DDOWNLOAD_ARTIFACTS=ON -Dtvm_utility_BACKEND=vulkan
```

## Troubleshooting

To work around a bug for the nvidia+mesa libraries combination in the case of Vulkan, X forwading
is needed in the docker container.
- When launching Autoware locally, `ade` will take care of it with an `xhost` command.
- When launching Autoware on a remote machine over ssh, `ssh -X` is needed and the hostname of the
  container must match the one of the host. This can be done by editing the `.aderc` file in use
  and adding "-h ${YOUR_HOSTNAME}" to the `ADE_DOCKER_RUN_ARGS` variable.
