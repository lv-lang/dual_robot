from dataclasses import dataclass
from typing import Any, Optional

from sensor_msgs.msg import Image


SUPPORTED_INPUT_FORMATS = {"packed_nv12"}
SUPPORTED_RESIZE_MODES = {"direct_resize", "letterbox"}


@dataclass(frozen=True)
class ImagePreprocessConfig:
    input_width: int
    input_height: int
    input_format: str
    resize_mode: str


@dataclass(frozen=True)
class PreprocessedImage:
    image_msg: Image
    source_width: int
    source_height: int
    target_width: int
    target_height: int
    scale_x: float
    scale_y: float
    pad_x: float
    pad_y: float
    resize_mode: str
    input_format: str
    model_input: Optional[Any] = None


def preprocess_image(image_msg: Image, config: ImagePreprocessConfig) -> PreprocessedImage:
    """Prepare image metadata for the runtime wrapper.

    The first version intentionally keeps inference placeholder-only. The RDK X5
    integration point is here: convert ROS BGR/RGB images to the model input,
    typically resize or letterbox followed by packed NV12 for hbm_runtime.
    """
    if image_msg.width <= 0 or image_msg.height <= 0:
        raise ValueError("image width and height must be positive")
    if config.input_width <= 0 or config.input_height <= 0:
        raise ValueError("model input width and height must be positive")
    if config.input_format not in SUPPORTED_INPUT_FORMATS:
        raise ValueError(f"unsupported input_format: {config.input_format}")
    if config.resize_mode not in SUPPORTED_RESIZE_MODES:
        raise ValueError(f"unsupported resize_mode: {config.resize_mode}")

    source_width = int(image_msg.width)
    source_height = int(image_msg.height)
    target_width = int(config.input_width)
    target_height = int(config.input_height)

    if config.resize_mode == "letterbox":
        scale = min(target_width / source_width, target_height / source_height)
        resized_width = source_width * scale
        resized_height = source_height * scale
        return PreprocessedImage(
            image_msg=image_msg,
            source_width=source_width,
            source_height=source_height,
            target_width=target_width,
            target_height=target_height,
            scale_x=scale,
            scale_y=scale,
            pad_x=(target_width - resized_width) / 2.0,
            pad_y=(target_height - resized_height) / 2.0,
            resize_mode=config.resize_mode,
            input_format=config.input_format,
        )

    return PreprocessedImage(
        image_msg=image_msg,
        source_width=source_width,
        source_height=source_height,
        target_width=target_width,
        target_height=target_height,
        scale_x=target_width / source_width,
        scale_y=target_height / source_height,
        pad_x=0.0,
        pad_y=0.0,
        resize_mode=config.resize_mode,
        input_format=config.input_format,
    )
