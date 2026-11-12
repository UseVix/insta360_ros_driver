# CHANGES OF THIS FORK
- Dynamic parameters change works correctly
- Equirectangular node is now more efficient
- Perspective node have been added. You can control fov through parameters and camera orientation by publishing to: /camera_orientation/quaternion
- MediaSDK hardware-accelerated stitcher (`media_sdk_stitcher`) has been added for real-time 360° stitching and FlowState gyro stabilization
- Video decoders (`ffmpeg_decoder`, `gstreamer_decoder`) and encoder (`gstreamer_encoder`) have been added for flexible H.264 streaming over ROS topics or direct RTP/UDP

# insta360_ros_driver

A ROS driver for the Insta360 cameras. This driver is tested on Ubuntu 22.04 with ROS2 Humble. The driver has also been verified on the Insta360 X2 and X3 cameras. The following resolutions are available, all at 30 FPS.
- 3840 x 1920
- 2560 x 1280
- 2304 x 1152
- 1920 x 960

You can change [this line](https://github.com/ai4ce/insta360_ros_driver/blob/79588d9e0e9d029c3371d4095ea718daaf1e06fb/src/main.cpp#L126) to edit the resolution.

## Installation
To use this driver, you need to first have Insta360 SDK. Please apply for the SDK from the [Insta360 website](https://www.insta360.com/sdk/home). 

For additional instructions, see this [post](https://github.com/ai4ce/insta360_ros_driver/issues/10#issuecomment-3371481987).

**Note: Please make you use the latest SDK. This package works with the SDK posted after April 23, 2025**

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/ai4ce/insta360_ros_driver
cd ..
```
Then, the Insta360 libraries need to be installed as follows:
- add the `camera` and `stream` header files inside the `include` directory
- add the `libCameraSDK.so` library under the `lib` directory.

### GStreamer Dependencies
For H.264 video encoding and decoding via GStreamer, install the required development and plugin packages:
```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

### MediaSDK Installation (Optional for Hardware Stitching)
The MediaSDK package contains `ins::RealTimeStitcher`, which accepts the camera's H.264/H.265 access units and IMU data and returns stitched frames. Place the supplied Debian package at this exact path in the ROS package:
```bash
lib/libMediaSDK-dev-3.1.1.0-20250922_191110-amd64.deb
```
The CMake configuration extracts this package into the build directory automatically. To use an SDK installed elsewhere, set its prefix explicitly:
```bash
colcon build --symlink-install --cmake-args \
  -DINSTA360_MEDIA_SDK_ROOT=/path/to/MediaSDK/prefix
```

### Building the Workspace
Afterwards, install the other required dependencies and build:
```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Camera Setup
Before continuing, **make sure the camera is set to dual-lens mode**.

Additionally, **ensure the camera's USB mode is set to Android**:
1. On the camera, swipe down the screen to the main menu
2. Go to Settings -> General
3. Set USB Mode to **Android** (not Webcam or other modes)
4. This is required for the ROS driver to properly detect and communicate with the camera (see [Issue #4](https://github.com/ai4ce/insta360_ros_driver/issues/4))

The Insta360 requires sudo privilege to be accessed via USB. To compensate for this, a udev configuration can be automatically created that will only request for sudo once. The camera can thus be setup initially via:
```bash
cd ~/ros2_ws/src/insta360_ros_driver
./setup.sh
```
This creates a symlink based on the vendor ID of Insta360 cameras. The symlink, in this case `/dev/insta` is used to grant permissions to the usb port used by the camera.

![setup](docs/setup.png)

**Sometimes, this does not work (e.g. you see "device /dev/insta not found" or something similar). You can try entering the commands manually, since that sometimes sees success, especially for the first time.**
```bash
echo SUBSYSTEM=='"usb"', ATTR{manufacturer}=='"Arashi Vision"', SYMLINK+='"insta"', MODE='"0777"' | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/insta
```

## Nodes & CLI Usage
Every node in this package can be run individually from the command line using `ros2 run`. All nodes accept configuration files via `--params-file` from the `config/` directory or direct parameter overrides.

### 1. Camera Driver (`insta360_ros_driver`)
Connects directly to the Insta360 camera hardware over USB, synchronizes system time to the camera, initializes live video streaming, and broadcasts compressed video packets alongside raw inertial motion measurements and camera telemetry.

#### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| None | - | This node does not subscribe to any topics |

#### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/dual_fisheye/image/compressed_stream` | `insta360_ros_driver/msg/CameraCompressedStreamData` | Stream packets containing compressed H.264/H.265 payload, stream type, and lens index |
| `imu/data_raw` | `sensor_msgs/msg/Imu` | Raw 6-DOF IMU telemetry (angular velocity and linear acceleration) |
| `/camera/exposure_data` | `insta360_ros_driver/msg/ExposureData` | Camera exposure time and timestamp metadata |
| `/insta360/camera_preview_info` | `insta360_ros_driver/msg/CameraPreviewInfo` | Latched camera preview parameters, crop offsets, sweep time, and calibration info |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| None | - | - | Live stream parameters (e.g. resolution and bitrate) are configured in code (`src/main.cpp`) |

#### CLI Usage
```bash
ros2 run insta360_ros_driver insta360_ros_driver
```

---

### 2. FFmpeg Video Decoder (`ffmpeg_decoder`)
Decodes compressed H.264 video streams using FFmpeg (`libavcodec`) hardware or software decoding and produces standard uncompressed BGR image frames.

#### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `<compressed_topic>` | `sensor_msgs/msg/CompressedImage` or `insta360_ros_driver/msg/CameraCompressedStreamData` | Compressed video stream (type depends on `use_custom_stream_msg`) |

#### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `<uncompressed_topic>` | `sensor_msgs/msg/Image` | Decoded raw BGR image frames |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `use_custom_stream_msg` | `bool` | `false` | When `true`, subscribes to `CameraCompressedStreamData`; when `false`, subscribes to standard `CompressedImage` |
| `compressed_topic` | `string` | `"/equirectangular/image/h264"` | Input compressed video topic (defaults to `"/dual_fisheye/image/compressed_stream"` if `use_custom_stream_msg` is `true`) |
| `uncompressed_topic` | `string` | `"/equirectangular/image/decoded"` | Output uncompressed image topic |
| `skip_frame` | `int` | `0` | Number of frames to skip between decoded outputs (0 = decode all frames) |
| `i_frame_only` | `bool` | `false` | When `true`, decodes keyframes (I-frames) only to reduce computational overhead |

#### CLI Usage
```bash
# Run with default configuration from config file:
ros2 run insta360_ros_driver ffmpeg_decoder --ros-args \
  --params-file config/ffmpeg_decoder.yaml

# To decode raw dual-fisheye camera stream directly:
ros2 run insta360_ros_driver ffmpeg_decoder --ros-args \
  -p use_custom_stream_msg:=true \
  -p uncompressed_topic:=/dual_fisheye/image
```

---

### 3. GStreamer Video Decoder (`gstreamer_decoder`)
Decodes compressed H.264 video streams using a configurable GStreamer pipeline and outputs raw BGR images.

#### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `<compressed_topic>` | `sensor_msgs/msg/CompressedImage` or `insta360_ros_driver/msg/CameraCompressedStreamData` | Compressed video stream (type depends on `use_custom_stream_msg`) |

#### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `<uncompressed_topic>` | `sensor_msgs/msg/Image` | Decoded raw BGR image frames |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `use_custom_stream_msg` | `bool` | `false` | When `true`, subscribes to `CameraCompressedStreamData`; when `false`, subscribes to standard `CompressedImage` |
| `compressed_topic` | `string` | `"/equirectangular/image/h264"` | Input compressed video topic (defaults to `"/dual_fisheye/image/compressed_stream"` if `use_custom_stream_msg` is `true`) |
| `uncompressed_topic` | `string` | `"/equirectangular/image/decoded"` | Output uncompressed image topic |
| `pipeline` | `string` | `"appsrc name=source is-live=true format=time ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink sync=false"` | GStreamer pipeline string for video decoding |

#### CLI Usage
```bash
# Run with default configuration from config file:
ros2 run insta360_ros_driver gstreamer_decoder --ros-args \
  --params-file config/gstreamer_decoder.yaml

# To decode raw dual-fisheye camera stream directly:
ros2 run insta360_ros_driver gstreamer_decoder --ros-args \
  -p use_custom_stream_msg:=true \
  -p uncompressed_topic:=/dual_fisheye/image
```

---

### 4. GStreamer Video Encoder (`gstreamer_encoder`)
Encodes raw uncompressed video frames into H.264 video using GStreamer and transmits the compressed stream either over a ROS topic or as an RTP/UDP network stream.

#### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `<input_topic>` | `sensor_msgs/msg/Image` | Uncompressed BGR image stream to encode |

#### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `<output_topic>` | `sensor_msgs/msg/CompressedImage` | Encoded H.264 video frames (published when `transport` is `"ros"`) |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `input_topic` | `string` | `"/equirectangular/image"` | Source uncompressed image topic |
| `output_topic` | `string` | `"/equirectangular/image/h264"` | Destination ROS compressed topic (when `transport` is `"ros"`) |
| `transport` | `string` | `"ros"` | Transmission protocol (`"ros"` or `"udp"`) |
| `pipeline` | `string` | `"appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=4000 ! rtph264pay pt=96 ! udpsink host=192.168.1.50 port=5000"` | GStreamer pipeline used when `transport` is `"udp"` |
| `ros_pipeline` | `string` | `"appsrc name=ros_source is-live=true ! videoconvert ! x264enc tune=zerolatency ! h264parse ! appsink name=ros_sink sync=false"` | GStreamer pipeline used when `transport` is `"ros"` |
| `fps` | `double` | `30.0` | Target encoding frame rate |

#### CLI Usage
```bash
# Default ROS topic streaming:
ros2 run insta360_ros_driver gstreamer_encoder --ros-args \
  --params-file config/gstreamer_encoder.yaml

# Direct RTP/UDP streaming override:
ros2 run insta360_ros_driver gstreamer_encoder --ros-args \
  --params-file config/gstreamer_encoder.yaml \
  -p transport:=udp
```

---

### 5. MediaSDK Hardware Stitcher (`media_sdk_stitcher`)
Performs real-time hardware-accelerated stitching of dual-fisheye video packets and IMU motion data into stabilized 360° equirectangular panoramas using the Insta360 MediaSDK library. It also estimates and outputs the camera's 3D stabilization orientation quaternion.

#### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `<input_video_topic>` | `insta360_ros_driver/msg/CameraCompressedStreamData` | Compressed camera stream packets containing dual-fisheye frames |
| `<input_imu_topic>` | `sensor_msgs/msg/Imu` | Raw 6-DOF IMU telemetry used for FlowState gyro stabilization |
| `/camera/exposure_data` | `insta360_ros_driver/msg/ExposureData` | Exposure metadata for image stitching illumination correction |
| `/insta360/camera_preview_info` | `insta360_ros_driver/msg/CameraPreviewInfo` | Camera crop parameters, optical offset, and timing metadata |

#### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `<output_topic>` | `sensor_msgs/msg/Image` | Stitched 360° equirectangular panorama image |
| `/equirectangular/flowstate_orientation` | `geometry_msgs/msg/QuaternionStamped` | Camera 3D stabilization orientation quaternion |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `input_video_topic` | `string` | `"/dual_fisheye/image/compressed_stream"` | Topic for incoming compressed video stream packets |
| `input_imu_topic` | `string` | `"/imu/data_raw"` | Topic for incoming IMU telemetry |
| `output_topic` | `string` | `"/equirectangular/image"` | Topic for stitched equirectangular output images |
| `output_width` | `int` | `1920` | Output panorama width in pixels |
| `output_height` | `int` | `960` | Output panorama height in pixels |
| `stitch_type` | `string` | `"dynamicstitch"` | Stitching algorithm (`dynamicstitch`, `template`, `optflow`, `aistitch`) |
| `enable_flowstate` | `bool` | `true` | Enables FlowState stabilization using IMU telemetry |
| `enable_direction_lock` | `bool` | `false` | Locks panorama orientation along the forward heading direction |
| `enable_deflicker` | `bool` | `false` | Suppresses exposure flicker between frames |
| `enable_defringe` | `bool` | `false` | Corrects chromatic lens fringing artifacts |

#### CLI Usage
```bash
ros2 run insta360_ros_driver media_sdk_stitcher --ros-args \
  --params-file config/media_sdk_stitcher.yaml
```

##### FlowState Orientation & Internal Stabilizer Offset (`find_stabilizer_offset.py`)
MediaSDK computes 3D camera stabilization orientations internally via `arvrender::SequenceStabilizer`. Because the offset is private to `libMediaSDK.so`, upgrading or changing the MediaSDK `.deb` package can change this memory offset.
To detect the offset:
```bash
# Direct execution (auto-detects libMediaSDK.so):
./src/insta360_ros_driver/scripts/find_stabilizer_offset.py
# Or via ROS 2:
ros2 run insta360_ros_driver find_stabilizer_offset.py
```
If the detected offset changes from default `0x260` (e.g. `0x270`), update the hex value in `src/media_sdk_stitcher.cpp` and rebuild.

---

### 6. Dual-Fisheye to Equirectangular Stitcher (`dual_fisheye2equirectangular_cpp`)
Stitches uncompressed dual-fisheye images into 360° equirectangular panoramas using OpenCV remapping on CPU or GPU with configurable lens alignment calibrations.

#### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/dual_fisheye/image` | `sensor_msgs/msg/Image` | Uncompressed dual-fisheye input image frame |

#### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/equirectangular/image` | `sensor_msgs/msg/Image` | Stitched equirectangular panorama image |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `cx_offset` | `double` | `0.0` | Horizontal optical center shift for fisheye lenses in pixels |
| `cy_offset` | `double` | `0.0` | Vertical optical center shift for fisheye lenses in pixels |
| `crop_size` | `int` | `0` | Crop region diameter in pixels (0 = auto-computed from image dimensions) |
| `translation` | `double[]` | `[0.0, 0.0, -0.105]` | Extrinsic translation vector `[x, y, z]` between lenses in meters |
| `rotation_deg` | `double[]` | `[-0.5, 0.0, 1.1]` | Extrinsic Euler rotation angles `[roll, pitch, yaw]` in degrees |
| `gpu` | `bool` | `true` | Enables GPU-accelerated remap (falls back to CPU if OpenCV CUDA is unavailable) |
| `out_width` | `int` | `1920` | Width of output equirectangular image in pixels |
| `out_height` | `int` | `960` | Height of output equirectangular image in pixels |

#### CLI Usage
```bash
ros2 run insta360_ros_driver dual_fisheye2equirectangular_cpp --ros-args \
  --params-file config/dual_fisheye2equirectangular.yaml
```

---

### 7. Dual-Fisheye to Perspective View (`dual_fisheye2perspective_cpp`)
Generates a virtual rectilinear perspective camera viewport directly from uncompressed dual-fisheye images with configurable field of view and real-time viewing orientation control.

#### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/dual_fisheye/image` | `sensor_msgs/msg/Image` | Uncompressed dual-fisheye input image frame |
| `/camera_orientation/quaternion` | `geometry_msgs/msg/Quaternion` | Orientation quaternion to dynamically steer the perspective camera direction |

#### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/perspective/image` | `sensor_msgs/msg/Image` | Rectilinear perspective view image |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `cx_offset` | `double` | `0.0` | Horizontal optical center shift for fisheye lenses in pixels |
| `cy_offset` | `double` | `0.0` | Vertical optical center shift for fisheye lenses in pixels |
| `crop_size` | `int` | `0` | Crop region diameter in pixels (0 = auto-computed from image dimensions) |
| `translation` | `float[]` | `[0.0, 0.0, -0.105]` | Extrinsic translation vector `[x, y, z]` between lenses in meters |
| `rotation_deg` | `float[]` | `[-0.5, 0.0, 1.1]` | Extrinsic Euler rotation angles `[roll, pitch, yaw]` in degrees |
| `gpu` | `bool` | `true` | Enables GPU-accelerated remap |
| `out_width` | `int` | `1920` | Width of output perspective image in pixels |
| `out_height` | `int` | `960` | Height of output perspective image in pixels |
| `horizontal_fov` | `double` | `120.0` | Horizontal field of view in degrees |
| `vertical_fov` | `double` | `81.7867893` | Vertical field of view in degrees |

#### CLI Usage
```bash
ros2 run insta360_ros_driver dual_fisheye2perspective_cpp --ros-args \
  --params-file config/dual_fisheye2perspective.yaml
```

---

### 8. Equirectangular to Perspective View (`equirectangular2perspective_cpp`)
Extracts a rectilinear perspective virtual camera view from a stitched 360° equirectangular panorama with configurable field of view and dynamic orientation control.

#### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/equirectangular/image` | `sensor_msgs/msg/Image` | Stitched equirectangular panorama image |
| `/camera_orientation/quaternion` | `geometry_msgs/msg/Quaternion` | Orientation quaternion to dynamically steer the perspective viewport direction |

#### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/perspective/image` | `sensor_msgs/msg/Image` | Extracted rectilinear perspective image |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `out_width` | `int` | `1920` | Width of output perspective image in pixels |
| `out_height` | `int` | `960` | Height of output perspective image in pixels |
| `horizontal_fov` | `double` | `120.0` | Horizontal field of view in degrees |
| `vertical_fov` | `double` | `81.7867893` | Vertical field of view in degrees |

#### CLI Usage
```bash
ros2 run insta360_ros_driver equirectangular2perspective_cpp --ros-args \
  --params-file config/equirectangular2perspective.yaml
```

---

### 9. IMU Orientation Filter (`imu_filter_madgwick`)
Fuses 6-DOF IMU angular rates and linear accelerations using the Madgwick AHRS algorithm to estimate camera orientation, and optionally broadcasts spatial coordinate transforms.

#### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `imu/data_raw` | `sensor_msgs/msg/Imu` | Raw IMU measurements from the camera driver |
| `imu/mag` | `sensor_msgs/msg/MagneticField` | Magnetic field measurements (optional, only used if `use_mag` is `true`) |

#### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `imu/data` | `sensor_msgs/msg/Imu` | Filtered IMU message containing estimated orientation quaternion |
| `/tf` | `tf2_msgs/msg/TFMessage` | Coordinate frame transform between `fixed_frame` and child frame (when `publish_tf` is `true`) |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `gain` | `double` | `0.1` | Filter gain determining orientation responsiveness vs. noise rejection |
| `zeta` | `double` | `0.0` | Gyro drift bias calculation parameter |
| `world_frame` | `string` | `"nwu"` | World frame convention (`"nwu"`, `"enu"`, `"ned"`) |
| `fixed_frame` | `string` | `"imu_stabilized"` | Reference coordinate frame ID for published TF transforms |
| `publish_tf` | `bool` | `true` | Whether to broadcast orientation transform to `/tf` |
| `reverse_tf` | `bool` | `false` | Reverses parent/child direction of broadcast TF transforms |
| `use_mag` | `bool` | `false` | Enables fusion of magnetometer data |
| `stateless` | `bool` | `false` | Operates in stateless mode without past history |
| `constant_dt` | `double` | `0.0` | Overrides time delta with a constant value (0.0 = compute from message timestamps) |
| `publish_debug_topics` | `bool` | `false` | Enables debug telemetry topics |
| `orientation_stddev` | `double` | `0.0` | Standard deviation assigned to estimated orientation covariance |

#### CLI Usage
```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
  --params-file config/imu_filter.yaml
```

---

### 10. Webcam Equirectangular Node (`webcam_equirectangular`)
Captures video frames from standard UVC webcam devices, verifies 2:1 aspect ratio, and outputs raw equirectangular video frames.

#### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| None | - | This node does not subscribe to any topics |

#### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `<topic>` | `sensor_msgs/msg/Image` | Captured raw equirectangular video frames |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `device` | `string` | `"/dev/video0"` | Path to Video4Linux webcam device node |
| `width` | `int` | `2880` | Frame capture width in pixels |
| `height` | `int` | `1440` | Frame capture height in pixels |
| `fps` | `double` | `30.0` | Frame capture rate in FPS |
| `topic` | `string` | `"/equirectangular/image"` | Output ROS image topic name |
| `pixel_format` | `string` | `"MJPG"` | V4L2 pixel capture format (`"MJPG"` or `"YUYV"`) |

#### CLI Usage
```bash
# Query available resolutions and formats:
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0

# Run webcam node:
ros2 run insta360_ros_driver webcam_equirectangular --ros-args \
  --params-file config/webcam_equirectangular.yaml
```

---

### 11. Extrinsic Calibration Tool (`calibrate_cpp.py`)
Provides an interactive graphical calibration interface to visually tune the extrinsic calibration offsets (`cx_offset`, `cy_offset`, `crop_size`, `translation`, `rotation_deg`) of the projection stitcher.

#### CLI Usage
```bash
# 1. Run camera driver and decoder:
ros2 run insta360_ros_driver insta360_ros_driver
ros2 run insta360_ros_driver ffmpeg_decoder --ros-args \
  -p use_custom_stream_msg:=true \
  -p uncompressed_topic:=/dual_fisheye/image

# 2. Launch interactive calibration GUI targeting stitcher node:
ros2 run insta360_ros_driver calibrate_cpp.py \
  --ros-args -p target_node:=dual_fisheye2equirectangular_node
```
The tool prints the resulting YAML configuration block directly to the terminal when finished:
```yaml
dual_fisheye2equirectangular_node:
  ros__parameters:
    cx_offset: 0.0
    cy_offset: 0.0
    crop_size: 960
    translation: [0.0, 0.0, -0.105]
    rotation_deg: [-0.5, 0.0, 1.1]
    gpu: true
    out_width: 1920
    out_height: 960
```

---

## Launch Files
Instead of running nodes individually, the package provides complete launch files for bringup and stream reception.

### 1. Camera Bringup (`bringup.launch.xml`)
Brings up the camera driver, stitcher, IMU orientation filter, perspective projection nodes, and video stream encoder in a single command.

```bash
ros2 launch insta360_ros_driver bringup.launch.xml
```

![bringup](docs/bringup_rqt.png)

#### Launch Arguments & Configuration
All nodes in `bringup.launch.xml` consistently load parameters from `.yaml` files in the `config/` directory. Each node can be toggled on/off, and its configuration file path can be customized via launch arguments:

| Argument | Default | Description |
| :--- | :--- | :--- |
| `media_sdk_stitcher` | `true` | Hardware-accelerated MediaSDK real-time stitcher (`/equirectangular/image`) |
| `media_sdk_stitcher_config` | `config/media_sdk_stitcher.yaml` | Path to YAML config for `media_sdk_stitcher` |
| `gstreamer_encoder` | `true` | GStreamer video encoder and streaming node |
| `gstreamer_encoder_config` | `config/gstreamer_encoder.yaml` | Path to YAML config for `gstreamer_encoder` |
| `udp` | `false` | Transmission mode: `false` for ROS topic (`/equirectangular/image/h264`), `true` for direct RTP/UDP |
| `ffmpeg_decoder` | `false` | FFmpeg decoder for raw dual-fisheye frames (`/dual_fisheye/image`) |
| `ffmpeg_decoder_config` | `config/ffmpeg_decoder.yaml` | Path to YAML config for `ffmpeg_decoder` |
| `dual_fisheye2equirectangular` | `false` | CPU/OpenCV remap stitcher (`dual_fisheye2equirectangular_cpp`) |
| `dual_fisheye2equirectangular_config` | `config/dual_fisheye2equirectangular.yaml` | Path to YAML config for OpenCV stitcher |
| `dual_fisheye2perspective` | `true` | Perspective view from dual-fisheye frames |
| `dual_fisheye2perspective_config` | `config/dual_fisheye2perspective.yaml` | Path to YAML config for `dual_fisheye2perspective` |
| `equirectangular2perspective` | `false` | Perspective view from equirectangular frames |
| `equirectangular2perspective_config` | `config/equirectangular2perspective.yaml` | Path to YAML config for `equirectangular2perspective` |
| `imu_filter` | `true` | Madgwick IMU orientation filter (`imu_filter_madgwick`) |
| `imu_config` | `config/imu_filter.yaml` | Path to YAML config for IMU filter |

#### Bringup Examples:
- **Default (MediaSDK hardware stitching + ROS H.264 streaming):**
  ```bash
  ros2 launch insta360_ros_driver bringup.launch.xml
  ```
- **Direct RTP/UDP streaming:**
  ```bash
  ros2 launch insta360_ros_driver bringup.launch.xml udp:=true
  ```
- **Turn off video streaming:**
  ```bash
  ros2 launch insta360_ros_driver bringup.launch.xml gstreamer_encoder:=false
  ```
- **Enable raw dual-fisheye image decoding:**
  ```bash
  ros2 launch insta360_ros_driver bringup.launch.xml ffmpeg_decoder:=true
  ```
- **Use CPU OpenCV stitcher instead of MediaSDK:**
  ```bash
  ros2 launch insta360_ros_driver bringup.launch.xml media_sdk_stitcher:=false dual_fisheye2equirectangular:=true
  ```

---

### 2. Stream Reception (`receiver.launch.xml`)
Client-side launch file to receive and decode the video stream sent by `gstreamer_encoder`. It supports both ROS topic decoding and direct UDP display via the `udp` boolean switch:

| Argument | Default | Description |
| :--- | :--- | :--- |
| `udp` | `false` | `false` for ROS topic decoding via `gstreamer_decoder`, `true` for direct RTP/UDP reception |
| `gstreamer_decoder_config` | `config/gstreamer_decoder.yaml` | Path to YAML config for `gstreamer_decoder` |
| `udp_port` | `5000` | UDP port to listen on in UDP mode |
| `udp_pipeline` | `gst-launch-1.0 udpsrc ...` | Custom GStreamer command line executed in UDP mode |

#### Receiver Examples:
- **ROS topic mode (default):** Decodes `/equirectangular/image/h264` to `/equirectangular/image/decoded` using `gstreamer_decoder`:
  ```bash
  ros2 launch insta360_ros_driver receiver.launch.xml
  ```
- **Direct UDP mode:** Launches a `gst-launch-1.0` pipeline process via `<executable>` to display incoming UDP frames directly in `autovideosink`:
  ```bash
  ros2 launch insta360_ros_driver receiver.launch.xml udp:=true
  # With custom UDP port:
  ros2 launch insta360_ros_driver receiver.launch.xml udp:=true udp_port:=5000
  ```
- **Direct UDP reception from command line without ROS:**
  ```bash
  gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink
  ```

---

### 3. Alternative Python Pipeline (`working_pipeline.launch.py`)
An alternative python-based launch pipeline launching the driver, `ffmpeg_decoder` directly consuming the custom stream packets to decode `/dual_fisheye/image`, and OpenCV stitcher (`dual_fisheye2equirectangular_cpp`):
```bash
ros2 launch insta360_ros_driver working_pipeline.launch.py
```

---

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=ai4ce/insta360_ros_driver&type=Date)](https://star-history.com/#ai4ce/insta360_ros_driver&Date)
