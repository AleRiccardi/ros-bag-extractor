# RosBagExtractor

## Installation

Start by cloning the repo:

```bash
git clone git@github.com:AleRiccardi/ros-bag-extractor.git
```

Run the installation script and follow the instructions:

```bash
cd ros-bag-extractor/
./configs/scripts/sh/install.sh
```

## Usage

Use the following command to extract poses from a ROS bag file:

```bash
python src/rbe/main.py /path/to/the/file.bag --pose "/topic/name"
```

Here you can find more information on the type of data you can extract:

```bash
Usage: main.py [OPTIONS] PATH_BAGS

Options:
  --velodyne TEXT    Topic name of the point cloud messages
  --ouster TEXT      Topic name of the point cloud messages
  --hokuyo TEXT      Topic name of the point cloud messages
  --imu TEXT         Topic name of the IMU messages
  --coordinate TEXT  Topic name of the coordinate messages
  --image TEXT       Topic name of the image messages
  --nav TEXT         Topic name of the sbg ekf navigation system
  --quat TEXT        Topic name of the sbg ekf quaternion system
  --pose TEXT        Topic name of the poses
  --help             Show this message and exit.
```
