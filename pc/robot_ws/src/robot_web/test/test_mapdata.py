import os
import time

import pytest

from robot_web import mapdata


def write_pgm(path, pixels):
    path.write_bytes(b"P5\n2 1\n255\n" + bytes(pixels))


def test_default_map_prefers_real_competition_map():
    path = mapdata.find_map_yaml()

    assert path is not None
    assert path.name == "real_competition_map.yaml"


def test_map_payload_cache_invalidates_when_image_changes(tmp_path):
    pytest.importorskip("PIL")
    map_yaml = tmp_path / "lv_home.yaml"
    image = tmp_path / "lv_home.pgm"
    map_yaml.write_text(
        "\n".join(
            [
                "image: lv_home.pgm",
                "resolution: 0.03",
                "origin: [-9.99, -9.99, 0]",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
            ]
        ),
        encoding="utf-8",
    )
    write_pgm(image, [0, 255])

    first = mapdata.load_map_payload(map_yaml)
    time.sleep(0.01)
    write_pgm(image, [255, 0])
    now = time.time() + 1.0
    os.utime(image, (now, now))

    second = mapdata.load_map_payload(map_yaml)

    assert first["available"] is True
    assert second["available"] is True
    assert first["image_base64"] != second["image_base64"]


def test_map_payload_crops_unknown_margin_and_adjusts_origin(tmp_path):
    pytest.importorskip("PIL")
    map_yaml = tmp_path / "lv_home.yaml"
    image = tmp_path / "lv_home.pgm"
    map_yaml.write_text(
        "\n".join(
            [
                "image: lv_home.pgm",
                "resolution: 0.5",
                "origin: [10.0, 20.0, 0]",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
            ]
        ),
        encoding="utf-8",
    )
    image.write_bytes(
        b"P5\n6 5\n255\n"
        + bytes(
            [
                205, 205, 205, 205, 205, 205,
                205, 205, 254, 0, 205, 205,
                205, 205, 254, 0, 205, 205,
                205, 205, 205, 205, 205, 205,
                205, 205, 205, 205, 205, 205,
            ]
        )
    )

    payload = mapdata.load_map_payload(map_yaml, crop_unknown=True, crop_padding_px=0)

    assert payload["available"] is True
    assert payload["width"] == 2
    assert payload["height"] == 2
    assert payload["source_width"] == 6
    assert payload["source_height"] == 5
    assert payload["origin"] == [11.0, 21.0, 0.0]
    assert payload["crop"] == {
        "enabled": True,
        "x": 2,
        "y": 1,
        "width": 2,
        "height": 2,
    }
