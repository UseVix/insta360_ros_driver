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

Set the `video_resolution` parameter of the camera driver (see `config/insta360_ros_driver.yaml`) to change the resolution.

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
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-vaapi
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

#### Published Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/dual_fisheye/image/compressed_stream` | `compressed_stream_topic` | `insta360_ros_driver/msg/CameraCompressedStreamData` | Stream packets containing compressed H.264/H.265 payload, stream type, and lens index |
| `imu/data_raw` | `imu_topic` | `sensor_msgs/msg/Imu` | Raw 6-DOF IMU telemetry (angular velocity and linear acceleration) |
| `/camera/exposure_data` | `exposure_data_topic` | `insta360_ros_driver/msg/ExposureData` | Camera exposure time and timestamp metadata |
| `/insta360/camera_preview_info` | `camera_preview_info_topic` | `insta360_ros_driver/msg/CameraPreviewInfo` | Latched camera preview parameters, crop offsets, sweep time, and calibration info |

#### Services
| Service Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/insta360/clock_sync` | `clock_sync_service` | `insta360_ros_driver/srv/ClockSync` | Estimates the offset between the node's ROS clock (`get_clock()->now()`) and the camera's media clock. Available once live streaming has started |

On each request the driver queries `GetCameraMediaTime()` `clock_sync_samples` times and uses the query with the smallest round trip (the same method as the latency measurement's clock sync, which uses the host monotonic clock instead of the ROS clock). The response contains:

| Field | Type | Description |
| :--- | :--- | :--- |
| `success` | `bool` | `false` if the camera returned no valid media time |
| `message` | `string` | `ok`, or the reason for the failure |
| `rtt_ms` | `float64` | Round trip of the selected query (ms) |
| `camera_time_ms` | `int64` | Camera media time returned by the selected query (ms) |
| `offset_ms` | `float64` | ROS clock at the midpoint of the selected query minus `camera_time_ms` (ms) |

A camera timestamp converts to ROS time as `camera_ms + offset_ms`. The estimate is accurate to about ±`rtt_ms`/2.

```bash
ros2 service call /insta360/clock_sync insta360_ros_driver/srv/ClockSync
```

#### CLI Usage
```bash
ros2 run insta360_ros_driver insta360_ros_driver --ros-args \
  --params-file config/insta360_ros_driver.yaml
```

#### Stream, Exposure, Mode & Image Parameters
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `video_resolution` | `1920x960p30` | `3840x1920p30`, `2560x1280p30`, `2304x1152p30`, `1920x960p30`, `1440x720p30`, `960x480p30`, `3840x1920p60`, `2560x1280p60` (support varies by model/firmware) |
| `video_bitrate` | `524288` | Live stream bitrate in bit/s |
| `exposure_mode` | `shutter_priority` | `unchanged`, `auto`, `shutter_priority`, `iso_priority`, `manual`, `adaptive`, `full_auto` |
| `shutter_speed_denominator` | `120` | Shutter = 1/N s (`0` = do not set). Use `100` under 50 Hz lighting to avoid banding |
| `iso` | `0` | ISO for `manual`/`iso_priority` (`0` = do not set) |
| `ev_bias` | `0` | Exposure compensation |
| `video_iso_top_limit` | `0` | Upper ISO limit for auto ISO (`0` = do not set) |
| `video_sub_mode` | `unchanged` | Camera video mode: `unchanged`, `normal`, `hdr`, `pure`, `liveview`, `loop_recording`, `timelapse`, `timeshift`, `bullet_time`, `slow_motion`, `selfie`, `super`, `dash_cam`, `ptz`. The camera keeps it after the driver exits |
| `apply_image_settings` | `false` | Apply the `image_*` settings below to the live stream |
| `image_contrast` / `image_saturation` | `64` | 0..256 |
| `image_brightness` | `0` | -256..256 |
| `image_sharpness` | `3` | 0..6 |
| `image_white_balance` | `auto` | `auto`, `2700k`, `4000k`, `5000k`, `6500k`, `7500k` |

Exposure is applied to the live-stream mode before streaming starts. The applied values and the first exposure time the camera reports are logged; `/camera/exposure_data` shows the exposure time continuously.

A fixed short shutter mostly matters in low light: the exposure time is bounded by the frame period (33 ms at 30 fps), so it saves at most a few tens of milliseconds, but it prevents auto-exposure from lowering the frame rate in dim scenes, which adds much more latency.

#### Latency Measurement
Set `measure_latency:=true` to log statistics on the path from the camera to the `OnVideoData` callback for each video stream (`stream_index`). In the usual dual-fisheye mode both lenses come in one stream, so only `stream 0` is reported.

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `measure_latency` | `false` | Enable latency instrumentation |
| `latency_report_period_s` | `5.0` | Interval between statistics reports |
| `clock_sync_period_s` | `10.0` | Interval between camera/host clock offset re-estimations |
| `clock_sync_samples` | `30` | `GetCameraMediaTime()` queries per sync (sample with the smallest round trip wins). Also used by the `clock_sync` service |

Each report gives min/mean/p50/p99/max for:
- **cam->callback**: absolute latency, using the estimated clock offset. Its uncertainty is about ±(sync RTT/2 + 1 ms). It also depends on which moment the camera's timestamp marks (exposure or encode), so check it with an optical test.
- **inter-arrival**: host time between consecutive packets of the same stream (~33 ms at 30 fps).

```bash
ros2 run insta360_ros_driver insta360_ros_driver --ros-args \
  --params-file config/insta360_ros_driver.yaml -p measure_latency:=true
```

#### Stream Format Check
The driver checks whether the incoming stream matches the requested `video_resolution` and `video_bitrate`. It is on by default (`check_stream_format`).
- **Resolution:** read from the SPS inside the H.264/H.265 bitstream, logged once when streaming starts and again if it changes. A mismatch is logged as a warning.
- **Frame rate:** measured from the camera timestamps of consecutive frames.
- **Bitrate:** measured from the bytes received per second of camera time.

With `measure_latency:=true`, a `[stream]` line with all three is logged each report period:
```
[stream] stream 0 | resolution=1920x960 requested=1920x960 MATCH | fps=29.98 requested=30.00 MATCH | bitrate=530123 requested=524288 ratio=1.01 MATCH | sps_fps=30.00 frames=150 keyframes=5 max_packet_bytes=45123
```

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `check_stream_format` | `true` | Enable the check |
| `stream_check_fps_tolerance` | `0.10` | fps is `MATCH` within this fraction of the request |
| `stream_check_bitrate_tolerance` | `0.50` | bitrate is `MATCH` within this fraction, otherwise `BELOW`/`ABOVE` |

Encoders produce less than the requested bitrate on easy-to-compress scenes, so `BELOW` alone does not mean the setting is ignored. `ABOVE` does. The reliable test is comparing runs with different requested bitrates, which the sweep below does.

#### Latency Sweep
`latency_sweep.py` finds out which camera settings affect latency, then measures every combination of the ones that do. The settings it tests (the first value of each list is the baseline):

| Setting | Option | Default values |
| :--- | :--- | :--- |
| Exposure | `--exposures` | `auto shutter:30 shutter:60 shutter:100 shutter:120 shutter:240 shutter:500 shutter:1000` |
| Video sub-mode | `--sub-modes` | `normal hdr pure liveview` |
| Codec | `--codecs` | `h264 h265` (`current` = keep the camera's codec, no prompts) |
| Contrast / saturation | `--contrast` / `--saturation` | `64 0 256` |
| Brightness | `--brightness` | `0 -256 256` |
| Sharpness | `--sharpness` | `3 0 6` |
| White balance | `--white-balance` | `auto 2700k 7500k` |
| Resolution / bitrate | `--resolutions` / `--bitrates` | `1920x960p30` / `524288` (one value each, so not tested) |

1. **Screening** (24 runs, ~17 min with the defaults). Each setting is changed on its own while everything else stays at baseline. The baseline runs 3 times, spread over the phase, to measure run-to-run variation. A setting has an effect if it moves mean latency by more than the larger of 10 ms (`--min-effect-ms`) or 3× the baseline's variation (`--noise-factor`), or changes the frame rate by more than 10 % (`--fps-effect`). Values the camera refuses (it won't start, or it rejects the setting) are left out.
2. **Full sweep:** every combination of the settings that had an effect. All other settings stay at baseline, and runs already done in screening are reused. If the full sweep would need more than `--max-full-runs` runs (default 300), it doesn't start; narrow the value lists or raise the limit.

The codec can't be switched through the SDK. When a run needs the other codec, the script asks you to change it on the camera's menu and press Enter (`s` skips that codec, `q` stops), then checks the driver log to confirm the switch. Runs are grouped so you switch as few times as possible. Codecs are tested in the order of `--codecs`: H.264 first (the baseline), then H.265. If the camera is on H.265 when the sweep starts, you are asked to switch to H.264 before the first run. Without a terminal (e.g. stdin closed), that codec is marked unavailable.

When the sweep finishes, the camera is put back to the baseline sub-mode and image settings (`--no-restore` to skip).

```bash
ros2 run insta360_ros_driver latency_sweep.py --out ~/sweep --dry-run          # plan and time estimate
ros2 run insta360_ros_driver latency_sweep.py --out ~/sweep                    # screening + full sweep
ros2 run insta360_ros_driver latency_sweep.py --out ~/sweep --phase screen     # screening only
ros2 run insta360_ros_driver latency_sweep.py --out ~/sweep --summary-only     # print the report again
```

Outputs: `<out>.csv` (one row per run, with the phase, every setting, latency and the received stream format), `<out>_windows.csv` (one row per report window), `<out>_report.txt` (screening verdicts, lowest-latency configurations, stream format check) and `<out>_logs/`. Re-running the same command resumes; runs that did not finish cleanly are retried. Ctrl+C once stops after the current run, twice stops immediately.

---

### 2. FFmpeg Video Decoder (`ffmpeg_decoder`)
Decodes compressed H.264 video streams using FFmpeg (`libavcodec`) hardware or software decoding and produces standard uncompressed BGR image frames.

#### Subscribed Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/equirectangular/image/h264` | `compressed_topic` | `sensor_msgs/msg/CompressedImage` or `insta360_ros_driver/msg/CameraCompressedStreamData` | Compressed video stream (type depends on `use_custom_stream_msg`; defaults to `/dual_fisheye/image/compressed_stream` when `use_custom_stream_msg` is `true`) |

#### Published Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/equirectangular/image/decoded` | `uncompressed_topic` | `sensor_msgs/msg/Image` | Decoded raw BGR image frames |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `use_custom_stream_msg` | `bool` | `false` | When `true`, subscribes to `CameraCompressedStreamData`; when `false`, subscribes to standard `CompressedImage` |
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
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/equirectangular/image/h264` | `compressed_topic` | `sensor_msgs/msg/CompressedImage` or `insta360_ros_driver/msg/CameraCompressedStreamData` | Compressed video stream (type depends on `use_custom_stream_msg`; defaults to `/dual_fisheye/image/compressed_stream` when `use_custom_stream_msg` is `true`) |

#### Published Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/equirectangular/image/decoded` | `uncompressed_topic` | `sensor_msgs/msg/Image` | Decoded raw BGR image frames |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `use_custom_stream_msg` | `bool` | `false` | When `true`, subscribes to `CameraCompressedStreamData`; when `false`, subscribes to standard `CompressedImage` |
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
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/equirectangular/image` | `input_topic` | `sensor_msgs/msg/Image` | Uncompressed BGR image stream to encode |

#### Published Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/equirectangular/image/h264` | `output_topic` | `sensor_msgs/msg/CompressedImage` | Encoded H.264 video frames (published when `transport` is `"ros"`) |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
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

#### Timing GStreamer encoding/decoding
`scripts/gst_timing.sh` toggles GStreamer's built-in `latency` tracer in the current shell by setting or unsetting `GST_TRACERS` and `GST_DEBUG`. Every GStreamer process started from that shell afterwards (`gstreamer_decoder`, `gstreamer_encoder`, `media_sdk_stitcher`, `gst-launch-1.0`) logs to stderr how long each element (encoder, decoder, parser, ...) takes per buffer, plus the total time from `appsrc` to `appsink`. It must be **sourced**:
```bash
source scripts/gst_timing.sh   # on if off, off if on
```
Times are in nanoseconds. To get the mean time per element, save a node's output (e.g. `... 2> gst.log`) and run:
```bash
grep "element-latency, element-id" gst.log | grep -v format \
  | sed -E 's/.*element=\(string\)([^,]+),.*time=\(guint64\)([0-9]+).*/\1 \2/' \
  | awk '{n[$1]++; t[$1]+=$2} END {for (e in n) printf "%-24s %6d buffers  %8.3f ms\n", e, n[e], t[e]/n[e]/1e6}'
```
Nodes that are already running are not affected until they are restarted.

---

### 5. MediaSDK Hardware Stitcher (`media_sdk_stitcher`)
Performs real-time hardware-accelerated stitching of dual-fisheye video packets and IMU motion data into stabilized 360° equirectangular panoramas using the Insta360 MediaSDK library. It also estimates and outputs the camera's 3D stabilization orientation quaternion.

#### Subscribed Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/dual_fisheye/image/compressed_stream` | `input_video_topic` | `insta360_ros_driver/msg/CameraCompressedStreamData` | Compressed camera stream packets containing dual-fisheye frames |
| `/imu/data_raw` | `input_imu_topic` | `sensor_msgs/msg/Imu` | Raw 6-DOF IMU telemetry used for FlowState gyro stabilization |
| `/camera/exposure_data` | `exposure_data_topic` | `insta360_ros_driver/msg/ExposureData` | Exposure metadata for image stitching illumination correction |
| `/insta360/camera_preview_info` | `camera_preview_info_topic` | `insta360_ros_driver/msg/CameraPreviewInfo` | Camera crop parameters, optical offset, and timing metadata |

#### Published Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/equirectangular/image` | `output_topic` | `sensor_msgs/msg/Image` | Stitched 360° equirectangular panorama image |
| `/equirectangular/flowstate_orientation` | `orientation_topic` | `geometry_msgs/msg/QuaternionStamped` | Camera 3D stabilization orientation quaternion |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `output_width` | `int` | `1920` | Output panorama width in pixels |
| `output_height` | `int` | `960` | Output panorama height in pixels |
| `stitch_type` | `string` | `"dynamicstitch"` | Stitching algorithm (`dynamicstitch`, `template`, `optflow`, `aistitch`) |
| `enable_flowstate` | `bool` | `true` | Enables FlowState stabilization using IMU telemetry |
| `enable_direction_lock` | `bool` | `false` | Locks panorama orientation along the forward heading direction |
| `enable_deflicker` | `bool` | `false` | Suppresses exposure flicker between frames |
| `enable_defringe` | `bool` | `false` | Corrects chromatic lens fringing artifacts |
| `disable_stabilization` | `bool` | `true` | Patches libMediaSDK so that no stabilization rotation is applied to the output: the panorama stays fixed to the camera. `orientation_topic` still publishes the stabilizer quaternion (camera attitude). If the patch cannot be installed (e.g. a different SDK build), an error is logged and the output stays stabilized. Keep `enable_flowstate` `true`, since libMediaSDK crashes without the stabilizer |

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
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/dual_fisheye/image` | `input_topic` | `sensor_msgs/msg/Image` | Uncompressed dual-fisheye input image frame |

#### Published Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/equirectangular/image` | `output_topic` | `sensor_msgs/msg/Image` | Stitched equirectangular panorama image |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `cx_offset` | `double` | `0.0` | (dynamically reconfigurable) Horizontal optical center shift for fisheye lenses in pixels |
| `cy_offset` | `double` | `0.0` | (dynamically reconfigurable) Vertical optical center shift for fisheye lenses in pixels |
| `crop_size` | `int` | `0` | (dynamically reconfigurable) Crop region diameter in pixels (0 = auto-computed from image dimensions) |
| `translation` | `double[]` | `[0.0, 0.0, -0.105]` | (dynamically reconfigurable) Extrinsic translation vector `[x, y, z]` between lenses in meters |
| `rotation_deg` | `double[]` | `[-0.5, 0.0, 1.1]` | (dynamically reconfigurable) Extrinsic Euler rotation angles `[roll, pitch, yaw]` in degrees |
| `gpu` | `bool` | `true` | (dynamically reconfigurable) Enables GPU-accelerated remap (falls back to CPU if OpenCV CUDA is unavailable) |
| `out_width` | `int` | `1920` | (dynamically reconfigurable) Width of output equirectangular image in pixels |
| `out_height` | `int` | `960` | (dynamically reconfigurable) Height of output equirectangular image in pixels |

#### CLI Usage
```bash
ros2 run insta360_ros_driver dual_fisheye2equirectangular_cpp --ros-args \
  --params-file config/dual_fisheye2equirectangular.yaml
```

---

### 7. Dual-Fisheye to Perspective View (`dual_fisheye2perspective_cpp`)
Generates a virtual rectilinear perspective camera viewport directly from uncompressed dual-fisheye images with configurable field of view and real-time viewing orientation control.

#### Subscribed Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/dual_fisheye/image` | `input_topic` | `sensor_msgs/msg/Image` | Uncompressed dual-fisheye input image frame |
| `/camera_orientation/quaternion` | `camera_orientation_topic` | `geometry_msgs/msg/Quaternion` | Orientation quaternion to dynamically steer the perspective camera direction |

#### Published Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/perspective/image` | `output_topic` | `sensor_msgs/msg/Image` | Rectilinear perspective view image |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `cx_offset` | `double` | `0.0` | (dynamically reconfigurable) Horizontal optical center shift for fisheye lenses in pixels |
| `cy_offset` | `double` | `0.0` | (dynamically reconfigurable) Vertical optical center shift for fisheye lenses in pixels |
| `crop_size` | `int` | `0` | (dynamically reconfigurable) Crop region diameter in pixels (0 = auto-computed from image dimensions) |
| `translation` | `float[]` | `[0.0, 0.0, -0.105]` | (dynamically reconfigurable) Extrinsic translation vector `[x, y, z]` between lenses in meters |
| `rotation_deg` | `float[]` | `[-0.5, 0.0, 1.1]` | (dynamically reconfigurable) Extrinsic Euler rotation angles `[roll, pitch, yaw]` in degrees |
| `gpu` | `bool` | `true` | (dynamically reconfigurable) Enables GPU-accelerated remap |
| `out_width` | `int` | `1920` | (dynamically reconfigurable) Width of output perspective image in pixels |
| `out_height` | `int` | `960` | (dynamically reconfigurable) Height of output perspective image in pixels |
| `horizontal_fov` | `double` | `120.0` | (dynamically reconfigurable) Horizontal field of view in degrees |
| `vertical_fov` | `double` | `81.7867893` | (dynamically reconfigurable) Vertical field of view in degrees |

#### CLI Usage
```bash
ros2 run insta360_ros_driver dual_fisheye2perspective_cpp --ros-args \
  --params-file config/dual_fisheye2perspective.yaml
```

---

### 8. Equirectangular to Perspective View (`equirectangular2perspective_cpp`)
Extracts a rectilinear perspective virtual camera view from a stitched 360° equirectangular panorama with configurable field of view and dynamic orientation control.

#### Subscribed Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/equirectangular/image` | `input_topic` | `sensor_msgs/msg/Image` | Stitched equirectangular panorama image |
| `/camera_orientation/quaternion` | `camera_orientation_topic` | `geometry_msgs/msg/Quaternion` | Orientation quaternion to dynamically steer the perspective viewport direction |

#### Published Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/perspective/image` | `output_topic` | `sensor_msgs/msg/Image` | Extracted rectilinear perspective image |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `out_width` | `int` | `1920` | (dynamically reconfigurable) Width of output perspective image in pixels |
| `out_height` | `int` | `960` | (dynamically reconfigurable) Height of output perspective image in pixels |
| `horizontal_fov` | `double` | `120.0` | (dynamically reconfigurable) Horizontal field of view in degrees |
| `vertical_fov` | `double` | `81.7867893` | (dynamically reconfigurable) Vertical field of view in degrees |

#### CLI Usage
```bash
ros2 run insta360_ros_driver equirectangular2perspective_cpp --ros-args \
  --params-file config/equirectangular2perspective.yaml
```

---

### 9. Webcam Equirectangular Node (`webcam_equirectangular`)
Captures video frames from standard UVC webcam devices, verifies 2:1 aspect ratio, and outputs raw equirectangular video frames.

#### Published Topics
| Topic Name (Default) | Parameter | Type | Description |
| :--- | :--- | :--- | :--- |
| `/equirectangular/image` | `topic` | `sensor_msgs/msg/Image` | Captured raw equirectangular video frames |

#### Parameters
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `device` | `string` | `"/dev/video0"` | Path to Video4Linux webcam device node |
| `width` | `int` | `2880` | Frame capture width in pixels |
| `height` | `int` | `1440` | Frame capture height in pixels |
| `fps` | `double` | `30.0` | Frame capture rate in FPS |
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

### 10. Extrinsic Calibration Tool (`calibrate_cpp.py`)
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
| `webcam_equirectangular` | `false` | Capture pre-stitched equirectangular frames directly from camera via V4L2 webcam mode (`webcam_equirectangular`). Disables `insta360_ros_driver` and `media_sdk_stitcher`. |
| `webcam_equirectangular_config` | `config/webcam_equirectangular.yaml` | Path to YAML config for `webcam_equirectangular` |
| `camera_driver_config` | `config/insta360_ros_driver.yaml` | Path to YAML config for camera driver (`insta360_ros_driver`) |
| `media_sdk_stitcher` | `true` | Hardware-accelerated MediaSDK real-time stitcher (`/equirectangular/image`). Active only when `webcam_equirectangular` is `false`. |
| `media_sdk_stitcher_config` | `config/media_sdk_stitcher.yaml` | Path to YAML config for `media_sdk_stitcher` |
| `gstreamer_encoder` | `true` | GStreamer video encoder and streaming node |
| `gstreamer_encoder_config` | `config/gstreamer_encoder.yaml` | Path to YAML config for `gstreamer_encoder` |
| `udp` | `false` | Transmission mode: `false` for ROS topic (`/equirectangular/image/h264`), `true` for direct RTP/UDP |
| `ffmpeg_decoder` | `false` | FFmpeg video decoder node (decodes compressed H.264 streams to raw images) |
| `ffmpeg_decoder_config` | `config/ffmpeg_decoder.yaml` | Path to YAML config for `ffmpeg_decoder` |
| `dual_fisheye2equirectangular` | `false` | CPU/OpenCV remap stitcher (`dual_fisheye2equirectangular_cpp`) |
| `dual_fisheye2equirectangular_config` | `config/dual_fisheye2equirectangular.yaml` | Path to YAML config for OpenCV stitcher |
| `dual_fisheye2perspective` | `false` | Perspective view from dual-fisheye frames |
| `dual_fisheye2perspective_config` | `config/dual_fisheye2perspective.yaml` | Path to YAML config for `dual_fisheye2perspective` |
| `equirectangular2perspective` | `true` | Perspective view from equirectangular frames |
| `equirectangular2perspective_config` | `config/equirectangular2perspective.yaml` | Path to YAML config for `equirectangular2perspective` |
| `imu_filter` | `true` | Madgwick IMU orientation filter (`imu_filter_madgwick`) |
| `imu_config` | `config/imu_filter.yaml` | Path to YAML config for IMU filter |
| `decouple_flowstate` | `true` | Republishes the stitcher's `/equirectangular/flowstate_orientation` as `/camera_orientation/quaternion`, so the perspective nodes follow the orientation FlowState used and the view is decoupled from the stabilization rotation |
| `display_flowstate_orientation` | `true` | Republishes `/equirectangular/flowstate_orientation` as a `geometry_msgs/msg/PoseStamped` on `/pose_out` and publishes a static identity `map` -> `camera_frame` TF, so the orientation can be viewed in RViz2 |

> [!NOTE]
> **Camera Driver Mode vs. Webcam Mode:**
> - **Camera Driver Mode (Default, `webcam_equirectangular:=false`):**
>   Connects directly to the camera over USB using the Insta360 SDK (`insta360_ros_driver`), publishing raw dual-fisheye compressed stream packets and 6-DOF IMU telemetry. The `media_sdk_stitcher` stitches this stream into `/equirectangular/image`.
> - **Webcam Mode (`webcam_equirectangular:=true`):**
>   Captures pre-stitched 2:1 equirectangular frames directly from the camera via standard V4L2/UVC (`webcam_equirectangular`). Setting `webcam_equirectangular:=true` automatically turns off `insta360_ros_driver` and `media_sdk_stitcher` (for the stitcher to activate, both `media_sdk_stitcher:=true` AND `webcam_equirectangular:=false` must be set). Downstream nodes like `gstreamer_encoder` continue to stream the stitched equirectangular output seamlessly.

#### Bringup Examples:
- **Default (Camera USB SDK + MediaSDK hardware stitching + ROS H.264 streaming):**
  ```bash
  ros2 launch insta360_ros_driver bringup.launch.xml
  ```
- **Webcam Mode (direct on-camera stitched equirectangular frames via V4L2):**
  ```bash
  ros2 launch insta360_ros_driver bringup.launch.xml webcam_equirectangular:=true
  ```
- **Direct RTP/UDP streaming:**
  ```bash
  ros2 launch insta360_ros_driver bringup.launch.xml udp:=true
  ```
- **Turn off video streaming:**
  ```bash
  ros2 launch insta360_ros_driver bringup.launch.xml gstreamer_encoder:=false
  ```
- **Enable video stream decoding via FFmpeg:**
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
