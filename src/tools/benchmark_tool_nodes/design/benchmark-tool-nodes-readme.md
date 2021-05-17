README {#benchmark-tool-nodes-readme}
=============================

A benchmark for Autoware.auto nodes using different datasets.


## The KITTI dataset

The [KITTI](http://www.cvlibs.net/datasets/kitti/) dataset is a project from the
Karlsruhe Institute of Technology and the Toyota Technological Institute of
Chicago. It provides large datasets of recording from an instrumented vehicle
driving around the mid-size city of Karlsruhe, in rural areas and on highways.

The vehicle is a standard station wagon equipped with two high-resolution color
and grayscale video cameras, a Velodyne laser scanner and a GPS localization
system.

For each dataset evaluation metrics are provided, having in that way a baseline
to train or evaluate computer vision algorithms.

Datasets contain series of frames for camera, point cloud and gps/imu; for
each frame a text file containing labels. Folder structures are different
depending on the subject of the benchmark. Here follows some categories of
interest:

- Raw data
- 3D Object detection evaluation 2017

### Understand the velodyne format

Velodyne points are stored as Nx4 float matrix into a binary file.
The first 3 values are the coordinates x, y and z and the last value is the
reflectance.
The unit for the coordinates is meter (m).

Here follows a piece of code able to read the velodyne files:

```{cpp}
  int32_t num = 1000000;
  float *data = (float*)malloc(num*sizeof(float));

  // pointers
  float *px = data+0;
  float *py = data+1;
  float *pz = data+2;
  float *r = data+3;

  // load point cloud
  FILE *stream;
  stream = fopen ("file.bin","rb");
  num = fread(data,sizeof(float),num,stream)/4;
  for (int32_t i=0; i<num; i++) {

    // manage *px,*py,*pz,*r

    px+=4; py+=4; pz+=4; pr+=4;
  }
  fclose(stream);
```

### Raw data

Raw data section contains datasets of recordings on different categories (city,
residential, road, campus, person, calibration), each dataset is composed of:

- color and grayscale frame series at 0.5 Megapixels
- Velodyne pointclouds (100k points per frame)
- 3D GPS/IMU data (location, speed, acceleration, meta information,
  stored as text file)
- Calibration files
- 3D object tracklet labels in xml format (not available for all recordings)

Here follows the folder structure:

```
.
└── YYYY_MM_DD
    ├── YYYY_MM_DD_drive_0001_sync
    │   ├── image_00
    │   │   ├── data
    │   │   └── timestamps.txt
    │   ├── image_01
    │   │   ├── data
    │   │   └── timestamps.txt
    │   ├── image_02
    │   │   ├── data
    │   │   └── timestamps.txt
    │   ├── image_03
    │   │   ├── data
    │   │   └── timestamps.txt
    │   ├── oxts
    │   │   ├── data
    │   │   ├── dataformat.txt
    │   │   └── timestamps.txt
    │   ├── tracklet_labels.xml
    │   └── velodyne_points
    │       ├── data
    │       ├── timestamps_end.txt
    │       ├── timestamps_start.txt
    │       └── timestamps.txt
    ├── calib_cam_to_cam.txt
    ├── calib_imu_to_velo.txt
    └── calib_velo_to_cam.txt
```
* image_00 contains grayscale left camera frames
* image_01 contains grayscale right camera frames
* image_02 contains color left camera frames
* image_03 contains color right camera frames
* tracklet_labels.xml contains recognized objects with bounding boxes coordinates

### 3D Object detection evaluation 2017

The 3D Object detection dataset contains left and right camera frames and
velodyne point cloud data to perform object detection and recognition.
The benchmark is based on different classes of objects:

- car
- pedestrian
- cyclist

The aim of the 3D object detection evaluation is to train an algorithm and then
test it with the testing data, comparing the results with the baseline supplied
with the dataset.
The code for the comparison is supplied and it is written in C++.

This dataset is extracted from the raw data, taking random frames. The mapping
between the dataset and the source from raw data is available on the Kitti
website.

```
.
├── testing
│   ├── calib
│   │   ├── 000000.txt
│   │   ├── ...
│   │   └── 007517.txt
│   ├── image_2
│   │   ├── 000000.png
│   │   ├── ...
│   │   └── 007517.png
│   ├── image_3
│   │   ├── 000000.png
│   │   ├── ...
│   │   └── 007517.png
│   └── velodyne
│   │   ├── 000000.bin
│   │   ├── ...
│   │   └── 007517.bin
└── training
    ├── calib
    │   ├── 000000.txt
    │   ├── ...
    │   └── 007480.txt
    ├── image_2
    │   ├── 000000.png
    │   ├── ...
    │   └── 007480.png
    ├── image_3
    │   ├── 000000.png
    │   ├── ...
    │   └── 007480.png
    ├── label_2
    │   ├── 000000.txt
    │   ├── ...
    │   └── 007480.txt
    └── velodyne
        ├── 000000.bin
        ├── ...
        └── 007480.bin
```

- image_2 contains color left camera frames
- image_3 contains color right camera frames
- training/label_2 contains labels for the algorithm training (baseline or
  ground-truth)
- velodyne contains the point cloud data as bin files


#### 3d benchmark baseline data format

The baseline dataset has one object entry on each row consisting of 15 values
which are listed below in order:

- Class of the object {string} (car, pedestrian, cyclist)
- Truncated pixel ratio {float} ([0..1]), value -1 for undefined
- Occlusion {enum} (0 = visible, 1 = partly occluded, 2 = fully occluded,
  3 = unknown), value -1 for undefined
- Object observation angle {float} ([-pi..pi]), value -10 for undefined
- 2D bounding box 0-based coordinates, left (x1) {float}
- 2D bounding box 0-based coordinates, top (y1) {float}
- 2D bounding box 0-based coordinates, right (x2) {float}
- 2D bounding box 0-based coordinates, bottom (y2) {float}
- 3D bounding box coordinates, height {float}, value -1 for undefined
- 3D bounding box coordinates, width {float}, value -1 for undefined
- 3D bounding box coordinates, length {float}, value -1 for undefined
- 3D bounding box coordinates, location X {float}, value -1000 for undefined
- 3D bounding box coordinates, location Y {float}, value -1000 for undefined
- 3D bounding box coordinates, location Z {float}, value -1000 for undefined
- 3D bounding box coordinates, yaw angle {float}, value -10 for undefined

#### 3d benchmark output data format

The output that the benchmark expects is similar to the baseline data format, it
adds just one value that is the score. So the output is formed by 16 values.

- 15 values from baseline format
- score, it is the confidence in the detection, high is better {float} ([0..1])
