from dataclasses import dataclass
from typing import Any, List, Sequence

from robot_vision_bpu.preprocess import PreprocessedImage


@dataclass(frozen=True)
class RuntimeConfig:
    backend: str
    model_path: str
    priority: int
    bpu_cores: List[int]


@dataclass(frozen=True)
class RawDetection:
    class_id: int
    score: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float


class PlaceholderBpuRuntime:
    """Inference wrapper used until an RDK X5 model is selected."""

    backend = "placeholder"

    def __init__(self, config: RuntimeConfig) -> None:
        self.config = config

    def predict(self, preprocessed: PreprocessedImage) -> Sequence[RawDetection]:
        del preprocessed
        return []


class HbmRuntimeBpuRuntime:
    """RDK X5 hbm_runtime integration point.

    A concrete model adapter should wire packed NV12 input and decode model
    outputs before selecting backend=hbm_runtime in production configs.
    """

    backend = "hbm_runtime"

    def __init__(self, config: RuntimeConfig) -> None:
        if not config.model_path:
            raise ValueError("model_path is required for backend=hbm_runtime")
        try:
            import hbm_runtime  # type: ignore
        except ImportError as exc:
            raise RuntimeError("hbm_runtime is only expected on the RDK X5 runtime image") from exc

        self.config = config
        self._runtime = hbm_runtime.HB_HBMRuntime(config.model_path)
        self._model_name = self._runtime.model_names[0]
        self._input_names = self._runtime.input_names[self._model_name]
        self._output_names = self._runtime.output_names[self._model_name]
        self._runtime.set_scheduling_params(
            priority={self._model_name: config.priority},
            bpu_cores={self._model_name: config.bpu_cores},
        )

    def predict(self, preprocessed: PreprocessedImage) -> Sequence[RawDetection]:
        if preprocessed.model_input is None:
            raise RuntimeError("hbm_runtime requires preprocessed.model_input packed as NV12")

        inputs = {
            self._model_name: {
                self._input_names[0]: preprocessed.model_input,
            }
        }
        raw_outputs: Any = self._runtime.run(inputs)[self._model_name]
        del raw_outputs
        raise NotImplementedError("model-specific decode is not implemented yet")


def create_runtime(config: RuntimeConfig):
    if config.backend == "placeholder":
        return PlaceholderBpuRuntime(config)
    if config.backend == "hbm_runtime":
        return HbmRuntimeBpuRuntime(config)
    raise ValueError(f"unsupported inference backend: {config.backend}")
