#include <gtest/gtest.h>


#include <localization_nodes/localization_node.hpp>
#include "test_relative_localizer_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace autoware
{
namespace localization
{
namespace localization_nodes
{
class RelativeLocalizationNodeTest : public ::testing::Test
{
public:
  RelativeLocalizationNodeTest()
  {
    m_observation_msg.header.frame_id = m_obs_frame;
    m_map_msg.header.frame_id = m_map_frame;
    // output of `register_measurement` has the frame ID:
    // output_frame_id = observation.frame_id + initial_guess.frame_id
    // output_frame_id = observation.frame_id + (observation.frame_id + map.frame_id)
    m_expected_aggregated_frame = m_obs_frame + m_obs_frame + m_map_frame;
  }
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    // TODO: add assertions!
  }
  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  const std::string m_observation_topic{"test_obs"};
  const std::string m_map_topic{"test_map"};
  const std::string m_out_topic{"test_pose_out"};
  const uint32_t m_history_depth{10U};
  const std::string m_map_frame{"map"};
  const std::string m_obs_frame{"obs"};
  std::string m_expected_aggregated_frame;
  TestObservation m_observation_msg;
  TestMap m_map_msg;
};


TEST_F(RelativeLocalizationNodeTest, basic) {

  ////////////////////////////// Define lambdas

  /// Spin until tracker pointer reaches
  auto spin_until_tracker_match =
    [](auto & node_ptr, auto & ptr, auto msg_id, auto max_poll_iters) {
      for (auto iter = 0U; (iter < max_poll_iters) && (get_msg_id(*ptr) != msg_id); ++iter) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        rclcpp::spin_some(node_ptr);
      }
    };

  /////////////////////// initialize
  constexpr auto initial_ID = -1;
  auto cur_map_id = initial_ID;
  auto cur_obs_id = initial_ID;
  const auto max_poll_iters = 50U;
  auto callback_called = false;

  // Create pointers to inject into the node to track its state.
  auto observation_tracker_ptr = std::make_shared<TestObservation>();
  auto map_tracker_ptr = std::make_shared<TestMap>();
  // Tag the pointers with the initial id
  set_msg_id(*observation_tracker_ptr, initial_ID);
  set_msg_id(*map_tracker_ptr, initial_ID);

  // Initialize localizer node
  auto localizer_ptr = std::make_unique<MockRelativeLocalizer>(observation_tracker_ptr,
      map_tracker_ptr);

  auto localizer_node = std::make_shared<TestRelativeLocalizerNode>("TestNode", "",
      TopicQoS{m_observation_topic, rclcpp::SystemDefaultsQoS{}},
      TopicQoS{m_map_topic, rclcpp::SystemDefaultsQoS{}},
      TopicQoS{m_out_topic, rclcpp::SystemDefaultsQoS{}},
      MockInitializer{});

  localizer_node->set_localizer_(std::move(localizer_ptr));


  // Create mock observation and map publishers.
  const auto observation_pub = localizer_node->create_publisher<TestObservation>(
    m_observation_topic,
    m_history_depth);
  const auto map_pub = localizer_node->create_publisher<TestMap>(m_map_topic, m_history_depth);

  // Create a subscription to get the output from the localizer node. The callback compares the
  // ID of the output to the last published
  const auto pose_out_sub = localizer_node->create_subscription<PoseWithCovarianceStamped>(
    m_out_topic,
    rclcpp::QoS{rclcpp::KeepLast{m_history_depth}},
    [this, &callback_called, &cur_obs_id](PoseWithCovarianceStamped::ConstSharedPtr pose) {
      // Check that the result has the expected frame ID.
      // See the initiallization of m_expected_aggregated_frame for an explanation.
      EXPECT_EQ(pose->header.frame_id, m_expected_aggregated_frame);
      // Compare the ID to the id of the last published observation.
      EXPECT_EQ(get_msg_id(*pose), cur_obs_id);
      callback_called = true;
    });

  // Wait until publishers have a subscription available.
  wait_for_matched(map_pub);
  wait_for_matched(observation_pub);
  wait_for_matched(localizer_node->get_publisher());

  ////////////////////////// Test routine:
  // * publish a map on 0th and 5th iterations
  // * publish an observation on each iteration.
  // At each iteration, injected tracker pointers are used to confirm that the localizer node
  // receives the published messages and forwarded them to the correct places in the localizer.
  // At the end of each iteration, the output of the localizer node is checked for confirmation.
  for (auto i = 0U; i < 10U; ++i) {
    ASSERT_NE(i, TEST_ERROR_ID);
    // Only publish maps every 5th message, starting from 0.
    if (i % 5U == 0U) {
      cur_map_id = i;
      ASSERT_NE(cur_map_id, initial_ID);
      set_msg_id(m_map_msg, cur_map_id);
      map_pub->publish(m_map_msg);
    }
    // Wait until correct map is received by the localizer.
    spin_until_tracker_match(localizer_node, map_tracker_ptr, cur_map_id, max_poll_iters);

    // Confirm map is correct.
    EXPECT_EQ(get_msg_id(*map_tracker_ptr), cur_map_id);

    // Publish a new observation with a new ID
    cur_obs_id = i;
    ASSERT_NE(cur_obs_id, initial_ID);
    set_msg_id(m_observation_msg, cur_obs_id);
    observation_pub->publish(m_observation_msg);

    // Wait until correct observation is received by the localizer
    spin_until_tracker_match(localizer_node, observation_tracker_ptr, cur_obs_id, max_poll_iters);

    EXPECT_EQ(get_msg_id(*observation_tracker_ptr), cur_obs_id);

    for (auto iter = 0U; (iter < max_poll_iters) && !callback_called; ++iter) {
      rclcpp::spin_some(localizer_node);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    EXPECT_TRUE(callback_called);
    callback_called = false;

    EXPECT_FALSE(localizer_node->map_exception());
    EXPECT_FALSE(localizer_node->register_exception());
  }
}

TEST_F(RelativeLocalizationNodeTest, exception_handling) {

  ////////////////////////////// Define lambdas
  /// Spin until condition for node is met
  auto spin_until_condition =
    [](auto & node_ptr, auto checker, auto max_poll_iters) {
      for (auto iter = 0U; (iter < max_poll_iters) && !checker(node_ptr); ++iter) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        rclcpp::spin_some(node_ptr);
      }
    };

  /////////////////////// initialize
  constexpr auto initial_id = -1;
  constexpr auto valid_map_id = 0U;
  const auto max_poll_iters = 50U;

  // Create pointers to inject into the node to track its state.
  auto map_tracker_ptr = std::make_shared<TestMap>();
  set_msg_id(*map_tracker_ptr, initial_id);

  ASSERT_NE(initial_id, valid_map_id);
  ASSERT_NE(initial_id, TEST_ERROR_ID);
  ASSERT_NE(valid_map_id, TEST_ERROR_ID);

  // Initialize localizer node
  auto localizer_ptr = std::make_unique<MockRelativeLocalizer>(nullptr, map_tracker_ptr);
  auto localizer_node = std::make_shared<TestRelativeLocalizerNode>("TestNode", "",
      TopicQoS{m_observation_topic, rclcpp::SystemDefaultsQoS{}},
      TopicQoS{m_map_topic, rclcpp::SystemDefaultsQoS{}},
      TopicQoS{m_out_topic, rclcpp::SystemDefaultsQoS{}},
      MockInitializer{});

  localizer_node->set_localizer_(std::move(localizer_ptr));

  // Create mock observation and map publishers.
  const auto observation_pub = localizer_node->create_publisher<TestObservation>(
    m_observation_topic, m_history_depth);
  const auto map_pub = localizer_node->create_publisher<TestMap>(m_map_topic, m_history_depth);

  // Wait until publishers have a subscription available.
  wait_for_matched(map_pub);
  wait_for_matched(observation_pub);

  set_msg_id(m_map_msg, TEST_ERROR_ID);
  map_pub->publish(m_map_msg);
  // Wait until map exception occurs.
  spin_until_condition(localizer_node, [](auto loc_nd_ptr) {return loc_nd_ptr->map_exception();},
    max_poll_iters);
  EXPECT_TRUE(localizer_node->map_exception());

  set_msg_id(m_observation_msg, TEST_ERROR_ID);
  observation_pub->publish(m_observation_msg);
  // run until observation is attempted to be registered when no valid map exists.
  spin_until_condition(localizer_node, [](auto loc_nd_ptr) {
      return loc_nd_ptr->register_on_invalid_map();
    }, max_poll_iters);
  // no exception will be thrown despite the registration is bad because no map is set yet.
  EXPECT_FALSE(localizer_node->register_exception());
  // Confirm that a registration was received with no valid map.
  EXPECT_TRUE(localizer_node->register_on_invalid_map());

  // Now we will set the node to a state where it has a valid map and try again.
  set_msg_id(m_map_msg, valid_map_id);
  map_pub->publish(m_map_msg);
  spin_until_condition(localizer_node, [map_tracker_ptr, valid_map_id](auto &) {
      return get_msg_id(*map_tracker_ptr) == valid_map_id;
    }, max_poll_iters);

  observation_pub->publish(m_observation_msg);
  spin_until_condition(localizer_node, [](auto loc_nd_ptr) {
      return loc_nd_ptr->register_exception();
    }, max_poll_iters);
  EXPECT_TRUE(localizer_node->register_exception());
}

////////////////////////////////////////////////////////////////////////Implementations

void TestRelativeLocalizerNode::set_localizer_(std::unique_ptr<MockRelativeLocalizer> && localizer)
{
  set_localizer(std::forward<std::unique_ptr<MockRelativeLocalizer>>(localizer));
}

MockRelativeLocalizer::MockRelativeLocalizer(
  std::shared_ptr<TestMap> obs_ptr,
  std::shared_ptr<TestObservation> map_ptr)
: m_observation_tracking_ptr{obs_ptr}, m_map_tracking_ptr{map_ptr} {}

PoseWithCovarianceStamped MockRelativeLocalizer::register_measurement_impl(
  const TestObservation & msg, const Transform & transform_initial)
{
  if (get_msg_id(msg) == TEST_ERROR_ID) {
    throw TestRegistrationException{};
  }
  PoseWithCovarianceStamped pose;
  // The resulting frame id should contain observation's frame + initial guess' frame ID
  // So the result should be: obs_frame + obs_frame + map_frame
  pose.header.frame_id = msg.header.frame_id + transform_initial.header.frame_id;
  set_msg_id(pose, get_msg_id(msg));

  // Update the tracking pointer for notifying the test.
  if (m_observation_tracking_ptr) {
    *m_observation_tracking_ptr = msg;
  }
  return pose;
}

void MockRelativeLocalizer::set_map_impl(const TestMap & msg)
{
  if (get_msg_id(msg) == TEST_ERROR_ID) {
    throw TestMapException{};
  }
  m_map = msg;
  // Update the tracking pointer for notifying the test.
  if (m_map_tracking_ptr) {
    *m_map_tracking_ptr = msg;
  }
}

const std::string & MockRelativeLocalizer::map_frame_id() const noexcept
{
  return m_map.header.frame_id;
}

std::chrono::system_clock::time_point MockRelativeLocalizer::map_stamp() const noexcept
{
  return ::time_utils::from_message(m_map.header.stamp);
}

void TestRelativeLocalizerNode::on_bad_registration(std::exception_ptr eptr)
{
  try {
    std::rethrow_exception(eptr);
  } catch (TestRegistrationException &) {
    m_register_exception = true;
  }
}

void TestRelativeLocalizerNode::on_bad_map(std::exception_ptr eptr)
{
  try {
    std::rethrow_exception(eptr);
  } catch (TestMapException &) {
    m_map_exception = true;
  }
}

bool TestRelativeLocalizerNode::register_exception()
{
  return m_register_exception;
}
bool TestRelativeLocalizerNode::map_exception()
{
  return m_map_exception;
}

bool TestRelativeLocalizerNode::register_on_invalid_map()
{
  return m_register_on_invalid_map;
}

void TestRelativeLocalizerNode::on_observation_with_invalid_map(
  TestObservation::ConstSharedPtr)
{
  m_register_on_invalid_map = true;
}

// Return a transform that contains information regarding two frame ids.
// The resulting frame id should contain: obs_frame + map_frame.
Transform MockInitializer::guess(
  const tf2::BufferCore &, tf2::TimePoint stamp,
  const std::string & map_frame, const std::string & obs_frame)
{
  Transform transform;
  transform.header.stamp = ::time_utils::to_message(stamp);
  transform.header.frame_id = obs_frame + map_frame;
  return transform;
}


}  // namespace autoware
}  // namespace localization
}  // namespace localization_nodes
