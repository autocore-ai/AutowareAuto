Integration Testing {#integration-testing}
========

@tableofcontents

# Introduction {#integration-testing-introduction}

This article motivates developers to adopt integration testing by explaining how to write, run,
and evaluate the results of integration tests.


# Quick reference {#integration-testing-quick-reference}

1. [colcon](https://github.com/ros2/ros2/wiki/Colcon-Tutorial) is used to build and run test.
2. [launch testing](https://github.com/ros2/launch/tree/master/launch_testing) launches nodes and runs tests.
3. @ref testing-in-general describes the big picture of testing.


# Integration testing {#integration-testing-integration-testing}

An integration test is defined as the phase in software testing where individual software
modules are combined and tested as a group. Integration tests occur after unit tests, and before
validation tests.

The input to an integration test is a set of independent modules that have been unit tested. The set
of modules are tested against the defined integration test plan, and the output is a set of
properly integrated software modules that are ready for system testing.


# Value of integration testing {#integration-testing-value-of-integration-testing}

Integration tests determine if independently developed software modules work correctly
when the modules are connected to each other. In ROS 2, the software modules are called
nodes. As a special case, testing a single node can be referred to as component testing.

Integration tests help to find the following types of errors:

- Incompatible interaction between nodes, such as non-matching topics, different message types, or
incompatible QoS settings
- Reveal edge cases that were not touched with unit tests, such as a critical timing issue, network
communication delay, disk I/O failure, and many other problems that can occur in production
environments
- Using tools like `stress` and `udpreplay`, performance of nodes is tested with real data
or while the system is under high CPU/memory load, where situations such as `malloc` failures can be
detected

With ROS 2, it is possible to program complex autonomous-driving applications with a large number
of nodes. Therefore, a lot of effort has been made to provide an integration-test framework that
helps developers test the interaction of ROS2 nodes.


# Integration-test framework {#integration-testing-integration-test-framework}

A typical integration-test framework has three parts:

1. A series of executables with arguments that work together and generate outputs
2. A series of expected outputs that should match the output of the executables
3. A launcher that starts the tests, compares the outputs to the expected outputs,
and determines if the test passes

In Autoware.Auto, we use the [launch_testing](https://github.com/ros2/launch/tree/master/launch_testing) framework.


## Integration test with a single node: component test {#integration-testing-component-test}

The simplest scenario is a single node. In this case, the integration test is commonly referred to
as a component test.

To add a component test to an existing node, follow the example of the `ndt_mapper_nodes` package that has a single node in an executable named `ndt_mapper_node_exe`.

In `package.xml`, add

```{xml}
<test_depend>ros_testing</test_depend>
```

In `CMakeLists.txt`, add or modify the `BUILD_TESTING` section:

```{cmake}
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/single_node_launch.test.py
    TIMEOUT "30"
  )
endif()
```
The `TIMEOUT` argument is given in seconds; see  [here](https://github.com/ros2/ros_testing/blob/master/ros_testing/cmake/add_ros_test.cmake) for details.

Create the file `test/single_node_launch.test.py` taking the [launch_testing quick-start example](https://github.com/ros2/launch/tree/master/launch_testing#quick-start-example) as an example.

The essential content is to first import dependencies:

```{python}
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing

import os
import pytest
import unittest
```

Then create a launch description to launch the node under test:

```{python}
@pytest.mark.launch_test
def generate_test_description():

    ndt_mapper = Node(
        package='ndt_mapping_nodes',
        executable='ndt_mapper_node_exe',
        name='ndt_mapper_node',
        node_namespace='mapper',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('ndt_mapping_nodes'), 'param/test.param.yaml')
        ],
    )

    context = {'ndt_mapper': ndt_mapper}

    launch_description = LaunchDescription([
        ndt_mapper,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()])

    return launch_description, context
```

and finally the test condition. In this case, it is just a smoke test that ensures the node can be

1. launched with its default parameter file,
2. terminated with a standard `SIGTERM` signal,

so the test code is executed after the node executable has been shut down (`post_shutdown_test`):

```{python}
@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, ndt_mapper):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=ndt_mapper)
```


## Running the test

Continuing the example from above, first build

```{bash}
$ ade enter
ade$ cd AutowareAuto
ade$ colcon build --packages-up-to ndt_mapping_nodes
ade$ source install/setup.bash
```

then either execute the component test manually

```{bash}
ade$ ros2 test test/ndt_mapping_nodes_launch.test.py
```

or as part of testing the entire package:

```{bash}
ade$ colcon test --packages-select ndt_mapping_nodes
```

Verify that the test is executed; e.g.

```{bash}
ade$ colcon test-result --all --verbose
...
build/ndt_mapping_nodes/test_results/ndt_mapping_nodes/test_ndt_mapping_nodes_launch.test.py.xunit.xml: 1 test, 0 errors, 0 failures, 0 skipped
```

## Next steps {#integration-testing-next-steps}

The simple test described in @ref integration-testing-component-test can be extended in numerous directions:

### Testing the output of a node

To test while the node is running, create an [*active test*](https://github.com/ros2/launch/tree/foxy/launch_testing#active-tests) by adding a subclass of Python's `unittest.TestCase` to `*launch.test.py`. Some boilerplate code is required to access output by creating a node and a subscription to a particular topic; e.g.

```{python}
import unittest

class TestRunningDataPublisher(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node("test_node", context=cls.context)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown(context=cls.context)

    def setUp(self):
        self.msgs = []
        sub = self.node.create_subscription(
            msg_type=my_msg_type,
            topic="/info_test",
            callback=self._msg_received
        )
        self.addCleanup(self.node.destroy_subscription, sub)

    def _msg_received(self, msg):
        # Callback for ROS 2 subscriber used in the test
        self.msgs.append(msg)

    def get_message(self):
        startlen = len(self.msgs)

        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)

        try:
            # Try up to 60 s to receive messages
            end_time = time.time() + 60.0
            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                if startlen != len(self.msgs):
                    break

            self.assertNotEqual(startlen, len(self.msgs))
            return self.msgs[-1]
        finally:
            executor.remove_node(self.node)

    def test_message_content():
        msg = self.get_message()
        self.assertEqual(msg, "Hello, world")
```


### Running multiple nodes together

To run multiple nodes together, simply add more nodes to the launch description in  `*launch.test.py`.
The lidar stack has more elaborate examples on how to feed input and to test more than just the exit status of nodes; see [point_cloud_filter_transform_tf_publisher.test.py](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/perception/filters/point_cloud_filter_transform_nodes/test/point_cloud_filter_transform_tf_publisher.test.py) for details.
