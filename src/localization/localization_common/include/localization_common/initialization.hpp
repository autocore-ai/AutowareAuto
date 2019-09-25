#ifndef LOCALIZATION_COMMON__INITIALIZATION_HPP_
#define LOCALIZATION_COMMON__INITIALIZATION_HPP_

#include <geometry_msgs/msg/transform.hpp>
#include <tf2/buffer_core.h>
// probably include the motion model

namespace autoware
{
namespace localization
{
namespace localization_common
{

template <typename Derived>
class PoseInitializationBase{
    using PoseT = geometry_msgs::msg::Transform;

    PoseT guess_initial(const tf2::BufferCore &);
};

class VehicleOdometryPoseInitialization : public PoseInitializationBase<VehicleOdometryPoseInitialization>{
    using PoseT = geometry_msgs::msg::Transform;

    PoseT guess_initial(const tf2::BufferCore &);
};

}          // namespace autoware
}      // namespace localization
}  // namespace localization_common

#endif