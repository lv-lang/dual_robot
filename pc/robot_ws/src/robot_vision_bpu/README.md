# robot_vision_bpu

ROS2 Humble package for the robot1 RDK X5 BPU vision detector skeleton.

## Node

- Node: `/robot1/bpu_vision_detector`
- Executable: `bpu_detector`
- Launch: `robot1_bpu_detector.launch.py`

## Topic contract

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| Subscribe | `/robot1/camera/color/image_raw` | `sensor_msgs/msg/Image` | Gazebo camera now, real camera later. |
| Publish | `/robot1/vision/detections` | `std_msgs/msg/String` | JSON placeholder schema `robot_vision_bpu.detections.string.v1`. Replace with `robot_msgs/DetectionArray` or `vision_msgs` once the project message is fixed. |
| Publish | `/robot1/vision/debug_image` | `sensor_msgs/msg/Image` | Pass-through debug image for now. Controlled by `publish_debug_image`. |

This package does not publish velocity topics, task topics, or `/robot1/goal_pose`.

## Parameters

Config file: `config/robot1_bpu_detector.yaml`

- `image_topic`: default `/robot1/camera/color/image_raw`
- `detections_topic`: default `/robot1/vision/detections`
- `debug_image_topic`: default `/robot1/vision/debug_image`
- `publish_debug_image`: default `true`
- `backend`: default `placeholder`
- `model_path`: empty until an RDK X5 `.bin` model is selected
- `priority`: default `0`
- `bpu_cores`: default `[0]`
- `input_width`: default `640`
- `input_height`: default `640`
- `input_format`: default `packed_nv12`
- `resize_mode`: default `letterbox`
- `score_threshold`: default `0.25`
- `nms_threshold`: default `0.45`
- `max_detections`: default `100`
- `class_names`: default `["target"]`

## RDK X5 integration points

The first version uses `PlaceholderBpuRuntime`, so it has no model dependency and always returns an empty detection list.

When wiring real inference, use the `rdk_model_zoo` `rdk_x5` branch and Python `hbm_runtime` path. The intended flow is:

1. Convert ROS image data to the selected model color layout.
2. Resize or letterbox to the model input size.
3. Convert BGR/RGB to RDK X5 packed NV12 when required by the model.
4. Run `hbm_runtime.HB_HBMRuntime`.
5. Decode model outputs, run NMS, and scale boxes back to the source image.

Do not use legacy `bpu_infer_lib_x5` or `hobot_dnn.pyeasy_dnn` as the default path for new RDK X5 work.
