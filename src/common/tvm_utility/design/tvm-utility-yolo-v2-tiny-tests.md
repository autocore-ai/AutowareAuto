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

2. Compile the neural network model using TVM. A CLI utility program and
   instructions on how to compile can be found at:
   https://github.com/autowarefoundation/modelzoo/tree/master/scripts/tvm_cli

   The files produced by the tvm_cli must then be placed in
   `/artifacts/yolo_v2_tiny/`. These files are:

   - inference_engine_tvm_config.hpp
   - deploy_lib.so
   - deploy_graph.json
   - deploy_param.params

3. Start autoware docker environment.

4. Build and test

```sh
$ colcon build --packages-up-to tvm_utility
$ colcon test --packages-select tvm_utility
```

# GPU backend

Vulkan is supported by default by the tvm_vendor package.

In order to use a GPU backend with ade, Autoware.Auto provides a `.aderc-gpu` configuration file.

## Troubleshooting

To work around a bug for the nvidia+mesa libraries combination in the case of Vulkan, X forwading
is needed in the docker container.
- When launching Autoware locally, `ade` will take care of it with an `xhost` command.
- When launching Autoware on a remote machine over ssh, `ssh -X` is needed and the hostname of the
  container must match the one of the host. This can be done by editing the `.aderc` file in use
  and adding "-h ${YOUR_HOSTNAME}" to the `ADE_DOCKER_RUN_ARGS` variable.
