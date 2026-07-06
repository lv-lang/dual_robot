from yolo_web.usb_camera_web_server import build_mjpeg_frame


def test_build_mjpeg_frame_wraps_jpeg_bytes():
    jpeg = b'abc123'
    frame = build_mjpeg_frame(jpeg)
    assert frame.startswith(b'--frame\r\n')
    assert b'Content-Type: image/jpeg\r\n' in frame
    assert b'Content-Length: 6\r\n' in frame
    assert frame.endswith(jpeg + b'\r\n')
