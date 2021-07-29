Projection {#projection}
============

Projection of a 3D point onto the image plane can be estimated using the geometric proportions 
derived from the 3D location of the point and the intrinsics of the camera.

# Purpose / Use cases

The pixel correspondence of a 3D point can be estimated by projecting it to the respective 
image plane. This is useful for associating the geometric structures in the space with the ones 
on captured on an image.

# Design

The projection of a point  is done by applying the projection matrix \f$P\f$:

\f{aligned}{
\lambda \begin{bmatrix}
x_p\\
y_p\\
1
\end{bmatrix} = P  \begin{bmatrix}
X\\
Y\\
Z\\
1
\end{bmatrix}
\f}

\f$(X, Y, Z)\f$ are the the 3D coordinated of the point and \f$(x_p, y_p)\f$ are the pixel 
coordinates of the projection. \f$\lambda\f$ is the depth of the point with respect to the 
camera frame.


\f$P\f$ can be computed using the intrinsic matrix \f$K\f$ and the `ego-to-camera` transform 
\f$T\f$ as per the equation \f$P = KT\f$. Intrinsic matrix is defined as below:

\f{aligned}{
K = \begin{bmatrix}
f_x & s & o_x\\
0 & f_y & o_y\\
0 & 0 & 1
\end{bmatrix}
\f}

\f$f_x\f$ and \f$f_y\f$ are the focal length on the x and y axes in pixels. \f$s\f$ is the 
pixel skew factor. \f$o_x\f$ and \f$o_y\f$ are the pixel principal point offsets. These 
parameters can be measured by calibrating the specific camera that generates the images.

## Inner-workings / Algorithms

Projection is done by executing the following steps:

* Construct the Projection matrix using the camera transformation and intrinsics
* Project all of the vertices representing the 3D shape
   * Points behind the camera are filtered out
* Compute a convex hull surrounding the projected points.
* Find the intersection between the convex hull and the image canvas
* Return the intersecting convex shape as the projection on the image plane

## Inputs / Outputs / API

Inputs:
* Camera intrinsics
* Camera transformation
* `autoware_auto_msgs::msg::Shape` representing the 3D shape.

Outputs:
* A list of vertices representing the projection shape.

## Related issues

- #983: Integrate vision detections in object tracker 