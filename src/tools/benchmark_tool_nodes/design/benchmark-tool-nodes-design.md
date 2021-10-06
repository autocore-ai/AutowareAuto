benchmark_tool_nodes {#benchmark-tool-nodes-design}
=============================

This is the design document for the `benchmark_tool_nodes` package.


# Purpose / Use cases

Benchmark Tool is a ROS2 package created to compute performance metrics on other
nodes or groups of nodes. The various components of the tool can be used to
extend it with many datasets, metrics, and supported nodes.
The tool comes with a launch file for each supported node allowing easy
usage and reproducible behaviour.

# Design

The tool is designed to run some benchmark task on a "blackbox" system and
collect metrics to be processed.

## Benchmark pipeline

The pipeline is formed by two stages. The first stage is used to play the
dataset into the blackbox system and to collect the resulting output. As
part of this stage, the output will be formatted as required for the validation
and saved in the correct folder.
When the metric to be collected is the speed of the iteration, that metric will
be computed by observing the time elapsed from the published topic to the output
topic of the system.

The second stage is used to compute the metrics, collecting all the output files
from the previous stage and applying the computation needed for each single
metric.

Algorithms used for metrics computation are:

- Lidar obstacle detection accuracy
  - apply the code used in the kitti 3d benchmark SDK

- Lidar obstacle detection iteration speed
  - collect speed information computed for each frame and apply arithmetic mean

- Camera obstacle detection accuracy
  - To be defined

- Camera obstacle detection iteration speed
  - To be defined

- Multimodal localization iteration speed
  - To be defined

- NDT node iteration speed
  - To be defined

- Ground filter iteration speed
  - To be defined

- Local planner iteration speed
  - To be defined

### First stage

The first stage block diagram explains the components of the benchmark tool and
the interactions involved.

@image html images/benchmark-tool-nodes-first-stage.png "First stage design" width=50%

#### Benchmark task

This is the main component of the tool. It is supposed to manage the interaction
between the other components. It is meant to be extended with other components
that encapsulate all the specific details of each test.

List of components that extend the **Benchmark task**:

- 3D lidar detection task
- Camera detection task
- Multimodal localization task
- NDT node task
- Ground filter task
- Local planner task

Each specialized component should:

- Hold information about the dataset to be used
- Know to which topics is must publish/subscribe
- Know how to handle the output and format it with the right Output formatter

#### Dataset

The **Dataset** component should standardize differences among datasets,
exposing a unified interface to the other components. It is not supposed to
hold any specific information about the dataset, this work is left to
specialized components that extend the Dataset.

List of components that extend the **Dataset**:

- 3D benchmark dataset

Each specialized component should:

- Hold information about the dataset structure (path, files)
- Unify the access to the data through the Dataset component interface

#### Player

The **Player** component is used to play the data to a given topic. It is
responsible for using the right datatype for communication.
A component that uses the **Player** should provide the topic and the data
to be sent.

#### Time estimation

The **Time estimation** component measures the time elapsed between an input
topic and an output topic. The measurements sent by this component to the
**Output formatter** are in microseconds.
A component that uses the **Time estimation** block should provide the topics
to listen to.

#### Output formatter

The **Output formatter** component is a general block used to listen to the
output data. It should be extended with specialized components that hold
specific information on how to format the output data. Moreover, the generic
formatter prints the raw incoming data to file.

List of components that extend the **Output formatter**:

- Kitti format

Each specialized component should:

- subscribe to a given output topic
- format the output data
- save files using the correct folder structure

### Second stage

@image html images/benchmark-tool-nodes-second-stage.png "Second stage design" width=50%

#### Second stage computation

The **Second stage computation** component is a general block that should be
specialized by other components. The main task is to compute a metric starting
from the output log file saved in the previous stage.

List of components that extend the **Second stage computation**:

- Kitti 3D benchmark computation
- Speed benchmark computation

Each specialized component should:

- hold information about which file is used for the computation
- apply its own computation algorithm to extract the metric from the data
- provide an output of the metric that could be easily used in a CI environment


## How to launch

List of supported nodes and how to launch them:

- euclidean_cluster_nodes
  - `ros2 launch benchmark_tool_nodes euclidean_cluster_node_benchmark.launch.py`
- ray_ground_classifier_nodes
  - `ros2 launch benchmark_tool_nodes ray_ground_classifier_benchmark.launch.py`
- ray_ground_classifier_euclidean_cluster_nodes
  - `ros2 launch benchmark_tool_nodes ray_ground_classifier_euclidean_cluster_node_benchmark.launch.py`
- ndt_matching [1]
  - `ros2 launch benchmark_tool_nodes lidar_localization_benchmark.launch.py`

[1] @ref benchmark-tool-nodes-lidar-localization "See the lidar localization documentation"

## Supported datasets

List of supported datasets:

- Kitti object detection evaluation 2017 (Only lidar data for now)

@ref benchmark-tool-nodes-readme "See the readme for more information"

### How Kitti benchmark works

The KITTI vision benchmark suite is a project of Karlsruhe Institute of
Technology and Toyota Technological Institute at Chicago. It provides data from
high-resolution color and grayscale cameras, velodyne pointclouds, and GPS, both
in raw format or preprocessed. It also provides evaluation metrics. Their goal
is to complement existing benchmarks by providing real-world benchmarks with
novel difficulties to the community.

In particular, the benchmark tool takes advantage of the Kitti object detection
evaluation 2017 dataset that is composed of the data listed above and provides
also matlab and c++ code to compute the accuracy of the benchmarked node output
compared to the dataset ground truth.

## Supported metrics

List of supoprted metrics:

- precision (in the sense of kitti object evaluation 2017)
- speed of iteration
  - derived from the header of the received data or from an estimation between
    the time elapsed from published data and received output.

## Parameters

Here follows a description of the common parameters for the benchmark tool.
Depending on each node-specific launch file, there could be more parameters.

|Parameter| Type| Description|Default|
|----------|-----|--------|---|
|`task`|*string*|The name of the benchmark task to start|depends on the launch file|
|`dataset_path`|*string*|The path of the root folder of the dataset|`$(env HOME)/kitti_data/3d_bench/`|
|`input_topic`|*string*|The topic to publish the data|depends on the launch file|
|`output_topic`|*string*|The topic to listen for output data|depends on the launch file|
|`benchmarked_input_topic`|*string*|The input topic of the benchmarked node|depends on the launch file|
|`benchmarked_output_topic`|*string*|The output topic of the benchmarked node|depends on the launch file|
|`result_path`|*string*|The path to save benchmark results|`$(env HOME)/benchmark_tool_result/`|
|`node_start_delay`|*int*|The amount of seconds to wait before starting the benchmark tool node|`0`|
|`rosbag_record`|*boolean*|Record on rosbag the input and output topics during the benchmark|`false`|
|`rosbag_record_subfolder`|*string*|The subfolder on filesystem to save the rosbag record file, it must be a subfolder of `result_path`|""|
|`node_name`|*string*|The name of the running node, change it to start multiple instances of the node|`benchmark_tool_node`|
|`node_output`|*string*|Where to display running informations (`screen` or `log`)|`screen`|
|`force_end_at_frame_n`|*int*|Limit the number of played frames (-1 means unlimited)|depends on the launch file|
|`ros_info_record`|*boolean*|Record ROS node topology and bandwidth information during the benchmark|`false`|
|`sys_info_record`|*boolean*|Record system metrics during the benchmark (cpu time, I/O, memory)|`false`|
|`cyclone_dds_info_record`|*boolean*|Record Cyclone DDS metrics during the benchmark (throughput and latency)|`false`|

### Info record

The optional `*_info_record` parameters generate text files at the root of the
`result_path` directory. Those files can be processed manually by the user
afterwards. In the case of `cyclone_dds_info_record`, the user has the option
of using Cyclone DDS's `latency-test-plot` or `throughput-test-plot` scripts to
plot the results.

# Future extensions / Unimplemented parts

## How to expand the tool

The tool comes with several classes that compose a general benchmark task.
The root components are:

- benchmark_task
  - The base class for a task (BenchmarkTask). Extend it and implement
    the required methods to perform the benchmark on the node of interest.
    Only one class is allowed for modules in this package.
- dataset
  - The base class for a dataset. Extend it to add support for a new dataset.
- metric
  - The base class for a generic metric. Extend it to add new metric support.
- output_formatter
  - The base class for a generic output formatter. Its work is to receive the
    output from the benchmarked node and format and save those data with a given
    structure. Extend it when adding a new node to the benchmark because each
    node has different characteristics of the output data.
- player
  - The player class is used to publish the data to the benchmarked node.
    There are currently two different player implementations. There may not be
    a need to create another player.
- time_estimator
  - The time_estimator package contains classes to measure speed. Each class
    makes a measurement and publishes the result on a topic. There are two
    implementations of that component: one extracts the measure from the header
    of received data and the other measures the time elapsed between publishing
    the data and receiving the measured node's output. The time estimator
    therefore subscribes to those two topics.
    Another time estimator can be implemented here if needed.

To add a new launch file for a new task, use the available launch files as an example,
include the benchmarked node along with the required
arguments in the launch file, and then insert the benchmark tool node with any
modified common parameters (the parameters listed above).

The `task` argument is very important because it will point the node to the
specific implementation of the task being called in the launch file (example:
 `'ray_ground_classifier_task'` will point the benchmark tool to load the
ray_ground_classifier_task.py module and instantiate the only class inside,
a BenchmarkTask derived class).

Keep track of the name of the new BenchmarkTask derived class filename because,
when adding a new task, the filename (without extension) has to be passed
in the `task` argument.

## Autoware metrics

The metrics to be implemented by the benchmark tool are the following:

- Lidar obstacle detection
  - speed of the iteration
  - accuracy of the results
- Camera obstacle detection
  - speed of the iteration
  - accuracy of the results
- Multimodal localization
  - speed of the iteration
- NDT node
  - speed of the iteration
- Ground filter
  - speed of the iteration
- Local planner
  - speed of the iteration

# Subpages

- @subpage benchmark-tool-nodes-readme
- @subpage benchmark-tool-nodes-lidar-localization
