tracking_test_framework {#tracking_test_framework-package-design}
===========

This is the design document for the `tracking_test_framework` package.


# Purpose / Use cases

We need a data generation and testing framework for testing the object tracking module in 
AutowareAuto. The framework should enable the user to create test cases with easily definable specifications.

## Assumptions / Known limits
1. For now we are assuming that tracking happens in 2D. So the objects being tracked are assumed 
to be represented as simple 2D shapes.

2. The Line shape is represented with fixed end points and is not infinite. 

## API
The framework will provide APIs for generating data for the tracked objects to be fed into 
the unit tests for tracking algorithms. The following can be considered an initial draft to be 
refined upon : 

```{cpp}
/// \brief Pedestrian and Car can be created using the following APIs:
1. Pedestrian(const Eigen::Vector2f & starting_position,
   const autoware::common::types::float32_t starting_speed,
   const autoware::common::types::float32_t orientation_deg,
   const autoware::common::types::float32_t turn_rate_deg_per_sec)
2. Car(const Eigen::Vector2f & starting_position,
   const autoware::common::types::float32_t starting_speed,
   const autoware::common::types::float32_t orientation_deg,
   const autoware::common::types::float32_t turn_rate_deg_per_sec,
   const Eigen::Vector2f & size);

/// \brief Lidar can be created using the following APIs:
3. Lidar(const Eigen::Vector2f & position, const uint32_t number_of_beams,
   const autoware::common::types::float32_t max_range);
/// \brief Scene can be setup with Lidar and the Pedestrian and Car. The Pedestrian and Car are 
/// passed as unique_ptrs to TrackedObject class and appended to a 
/// std::vector<std::unique_ptr<TrackedObject>> :
4. Scene(const Lidar & lidar, const std::vector<std::unique_ptr<TrackedObject>> & objects);
/// \brief Intersections between the Lidar and TrackedObjects are returned in a 
/// DetectedDynamicObjectArray form :
5. autoware_auto_msgs::msg::DetectedDynamicObjectArray Scene::get_detected_objects_array(
  const bool closest_only) const
/// \brief Objects can be moved in the scene using:
6. void Scene::move_all_objects(const std::chrono::milliseconds dt_in_secs)
```

## Inner-workings / Algorithms
1. The framework provides a 2D shape generator interface called the `Shape` class which is used to 
represent the tracked objects in 2D.

2. This abstract class is extended by `Line`, `Rectangle` and 
`Circle` each of which are the programmatic representations of LiDAR ray, cars and pedestrians 
respectively. These subclasses implement their own version of `intersect_with_line` function 
which gives back the intersection points of the `Line` with the `Rectangle` and `Circle` thus 
mimicking behaviour of getting back tracked points on the objects when the rays from the sensor 
touch the surface of the objects.
   
3. To be able to initialize the `Line` and use it we need : 
   Starting point represented as a 2D vector [xs,ys] and ending point as a 2D vector[xe,ye].
   The line will have a direction and length determined by start and end points in the 
   coordinate frame.

3. To be able to initialize the `Rectangle` and use it we need :
   Center point represented as a 2D vector [xc,yc] and size(length * breadth) as a 2D vector[l,
   b] plus an orientation [angle] w.r.t. x axis in degrees represented as float. The Rectangle's 
   borders and corner points will be determined by these inputs provided and the borders are needed when we are 
   trying to get intersection points with the `Line`. 

4. To be able to initialize the `Circle` and use it we need :
   Center point represented as a 2D vector [xc,yc] and radius [r] represented as a float. 


# Future extensions / Unimplemented parts
1. Implement 3D shape generator interface and methods for getting the intersection points.
2. Methods to verify tracker output
3. Objects generation for other sensor modalities like Radar etc

# Related issues
<!-- Required -->
https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/886