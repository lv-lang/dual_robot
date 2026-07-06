from pathlib import Path

from yolo_web.usb_camera_web_server import DEFAULT_MODEL_PATH, DEFAULT_FIRE_SMOKE_MODEL_PATH, package_asset_path


def test_default_models_live_under_yolo_web_models():
    assert Path(DEFAULT_MODEL_PATH).name == 'box_camera_best_bayese_640x640_nv12.bin'
    assert Path(DEFAULT_FIRE_SMOKE_MODEL_PATH).name == 'fire_smoke_best_bayese_640x640_nv12.bin'
    assert Path(DEFAULT_MODEL_PATH) == Path(package_asset_path('models', 'box_camera_best_bayese_640x640_nv12.bin'))
