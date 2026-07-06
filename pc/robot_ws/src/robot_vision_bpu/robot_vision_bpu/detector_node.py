import json
from typing import Any, Dict, Iterable, List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from robot_vision_bpu.postprocess import Detection, PostprocessConfig, decode_detections
from robot_vision_bpu.preprocess import ImagePreprocessConfig, PreprocessedImage, preprocess_image
from robot_vision_bpu.runtime import RuntimeConfig, create_runtime


class BpuVisionDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("bpu_vision_detector", namespace="/robot1")
        self._declare_parameters()
        self._load_parameters()
        self._validate_topic_contract()

        self._runtime = create_runtime(self._runtime_config)
        self._detections_pub = self.create_publisher(String, self._detections_topic, 10)
        self._debug_pub = (
            self.create_publisher(Image, self._debug_image_topic, 10)
            if self._publish_debug_image
            else None
        )
        self._image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "robot_vision_bpu subscribed to %s, publishing %s"
            % (self._image_topic, self._detections_topic)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/robot1/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/robot1/vision/detections")
        self.declare_parameter("debug_image_topic", "/robot1/vision/debug_image")
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("backend", "placeholder")
        self.declare_parameter("model_path", "")
        self.declare_parameter("priority", 0)
        self.declare_parameter("bpu_cores", [0])

        self.declare_parameter("input_width", 640)
        self.declare_parameter("input_height", 640)
        self.declare_parameter("input_format", "packed_nv12")
        self.declare_parameter("resize_mode", "letterbox")

        self.declare_parameter("score_threshold", 0.25)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("max_detections", 100)
        self.declare_parameter("class_names", ["target"])

    def _load_parameters(self) -> None:
        self._image_topic = str(self.get_parameter("image_topic").value)
        self._detections_topic = str(self.get_parameter("detections_topic").value)
        self._debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self._publish_debug_image = bool(self.get_parameter("publish_debug_image").value)

        self._model_path = str(self.get_parameter("model_path").value)
        self._runtime_config = RuntimeConfig(
            backend=str(self.get_parameter("backend").value),
            model_path=self._model_path,
            priority=int(self.get_parameter("priority").value),
            bpu_cores=[int(core) for core in self.get_parameter("bpu_cores").value],
        )
        self._preprocess_config = ImagePreprocessConfig(
            input_width=int(self.get_parameter("input_width").value),
            input_height=int(self.get_parameter("input_height").value),
            input_format=str(self.get_parameter("input_format").value),
            resize_mode=str(self.get_parameter("resize_mode").value),
        )
        self._postprocess_config = PostprocessConfig(
            score_threshold=float(self.get_parameter("score_threshold").value),
            nms_threshold=float(self.get_parameter("nms_threshold").value),
            max_detections=int(self.get_parameter("max_detections").value),
            class_names=[str(name) for name in self.get_parameter("class_names").value],
        )

    def _validate_topic_contract(self) -> None:
        topics = [
            self._image_topic,
            self._detections_topic,
            self._debug_image_topic,
        ]
        for topic in topics:
            if topic.startswith("/") and not topic.startswith("/robot1/"):
                raise ValueError(f"topic must stay under /robot1: {topic}")

        forbidden_topic_names = {"cmd_vel", "cmd_vel_raw", "goal_pose"}
        detections_name = self._detections_topic.rstrip("/").rsplit("/", 1)[-1]
        debug_image_name = self._debug_image_topic.rstrip("/").rsplit("/", 1)[-1]
        if detections_name in forbidden_topic_names:
            raise ValueError(f"detections_topic is forbidden: {self._detections_topic}")
        if debug_image_name in forbidden_topic_names:
            raise ValueError(f"debug_image_topic is forbidden: {self._debug_image_topic}")

    def _on_image(self, image_msg: Image) -> None:
        try:
            preprocessed = preprocess_image(image_msg, self._preprocess_config)
            raw_detections = self._runtime.predict(preprocessed)
            detections = decode_detections(raw_detections, preprocessed, self._postprocess_config)

            output = String()
            output.data = json.dumps(
                self._build_detection_payload(preprocessed, detections),
                separators=(",", ":"),
                sort_keys=True,
            )
            self._detections_pub.publish(output)

            if self._debug_pub is not None:
                self._debug_pub.publish(image_msg)
        except Exception as exc:
            self.get_logger().error(f"failed to process image: {exc}")

    def _build_detection_payload(
        self,
        preprocessed: PreprocessedImage,
        detections: Iterable[Detection],
    ) -> Dict[str, Any]:
        image_msg = preprocessed.image_msg
        return {
            "schema": "robot_vision_bpu.detections.string.v1",
            "header": {
                "frame_id": image_msg.header.frame_id,
                "stamp": {
                    "sec": int(image_msg.header.stamp.sec),
                    "nanosec": int(image_msg.header.stamp.nanosec),
                },
            },
            "source_image": {
                "width": int(image_msg.width),
                "height": int(image_msg.height),
                "encoding": image_msg.encoding,
            },
            "preprocess": {
                "input_width": preprocessed.target_width,
                "input_height": preprocessed.target_height,
                "input_format": preprocessed.input_format,
                "resize_mode": preprocessed.resize_mode,
            },
            "runtime": {
                "backend": self._runtime_config.backend,
                "model_path": self._model_path,
            },
            "detections": [self._detection_to_dict(detection) for detection in detections],
        }

    @staticmethod
    def _detection_to_dict(detection: Detection) -> Dict[str, Any]:
        return {
            "class_id": detection.class_id,
            "class_name": detection.class_name,
            "score": detection.score,
            "bbox_xyxy": [
                detection.x_min,
                detection.y_min,
                detection.x_max,
                detection.y_max,
            ],
        }


def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    node = BpuVisionDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
