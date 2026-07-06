from yolo_web.usb_camera_web_server import build_index_html


def test_index_html_only_renders_video_stream():
    html = build_index_html().decode("utf-8")

    assert "/stream.mjpg" in html
    assert "arrivalOverlay" not in html
    assert "inspectionResultOverlay" not in html
    assert "temporaryAppReceiver" not in html
    assert "new WebSocket" not in html
    assert "已到达" not in html
    assert "临时 APP 接收" not in html
    assert "alert(" not in html
    assert "confirm(" not in html


def test_index_html_does_not_post_arrival_confirmation():
    html = build_index_html().decode("utf-8")

    assert "fetch('/arrival_confirm'" not in html
    assert "method: 'POST'" not in html


def test_index_html_has_no_app_event_metadata_renderer():
    html = build_index_html().decode("utf-8")

    assert "].join('\n');" not in html
    assert "].join('\\n');" not in html
