Spinnaker camera driver {#spinnaker-camera-driver}
=======================

# Why we implemented this feature

We need to be able to read from a GigE (Ethernet) camera (and potentially other) and publish images through ROS2 pipeline. This driver handles the communication to the Spinnaker SDK to allow getting images from their cameras.

# How it is organized

The main idea behind this implementation is that all the SDK-facing functionality is hidden from the user. This library wraps the [Spinnaker SDK](https://www.flir.com/products/spinnaker-sdk/).

There is a number of wrappers that wrap corresponding objects from the Spinnaker SDK. `CameraWrapper` wraps the `CameraPtr` and handles subscription to images as well as manual image retreival along with proper camera creation, configuration and destruction.

The instances of `CameraWrapper` are created in the `CameraListWrapper` that handles the camera creation by getting them from the `Spinnaker::CameraList`. It also handles proper destruction sequence of the cameras and the `Spinnaker::CameraList`.

Finally, the `SystemWrapper` handles the creation, storage and proper destruction of the `Spinnaker::SystemPtr`.

# How to use
Using this should be as simple as:
```c++
using autoware::drivers::camera::spinnaker::SystemWrapper;
SystemWrapper system;
// Some camera settings. Dummy for the sake of example.
CameraSettings settings{42, 42, 42.42, CameraSettings::kPixelFormatStr_RGB8};
auto& cameras = system.create_cameras(settings);
cameras.set_image_callback([](std::uint32_t camera_index, std::unique_ptr<sensor_msgs::msg::Image> msg) {
    // Do smth with the image.
});
cameras.start_capturing();
```

# A word on tests
This package is nearly not tested and there are reasons for it. The underlying SDK proves very hard to extend for testing purposes. It wraps most of its reference-counted pointers in `Spinnaker::BasePtr<T>` wrapper, which unfortunately is not polymorphic on type `T`, i.e., one cannot implicitly convert a `BasePtr<Derived>` into a `BasePtr<Base>`. This makes it hard (if not impossible) to mock these. Adding to this that these are usually returned by copy means that whenever we create a object of Spinnaker SDK within any of our classes we lose control over the object being created and cannot easily mock it. 

An alternative would be to template each of our wrapper classes on all the Spinnaker types used within them, but this would significantly hinder readability as well as would change the design of the class for the sole reason that these classes would be replaced with their mocks during testing, which does not seem like a great design decision. 
