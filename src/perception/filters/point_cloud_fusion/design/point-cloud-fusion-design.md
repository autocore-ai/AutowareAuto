point_cloud_fusion
=============

# Purpose / Use cases

We need a node that will fuse two or more pointclouds into a single pointcloud.

# Design

The node uses a `message_filters::Synchronizer` with a synchronization
policy of `message_filters::sync_policies::ApproximateTime` to synchronize
messages coming from separate subscriptions.


## Assumptions / Known limits

Pointcloud fusion is supported from up to 8 sources only. This limitation is due to the
 `message_filters` package. Additionally, for the `ApproximateTime`
  policy to work, there should be `N+1` messages in
 the queue in case `N` messages are desired to be fused. This limitation comes from the fact that
 the synchronizer needs a reference point to be able to group messages of approximately similar
 times together. See the [ros1 documentation](http://wiki.ros.org/message_filters/ApproximateTime)
 for more information.

## Inputs / Outputs / API

These nodes have the following basic structure:

Input:
- input topic names
- output topic name
- point cloud capacity


# Related issues

- #38 - Implement point cloud fusion
