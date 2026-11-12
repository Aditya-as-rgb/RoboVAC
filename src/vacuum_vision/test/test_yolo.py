import os

import numpy as np
import pytest

from vacuum_vision import yolo


def fake_output(boxes, size=640, classes=4, anchors=8400):
    """(1, 4 + classes, anchors) with the given (cx, cy, w, h, cls, conf) rows at the front."""
    out = np.zeros((1, 4 + classes, anchors), np.float32)
    for i, (cx, cy, w, h, c, conf) in enumerate(boxes):
        out[0, :4, i] = (cx, cy, w, h)
        out[0, 4 + c, i] = conf
    return out


def test_letterbox_640x480_pads_top_and_bottom():
    img = np.full((480, 640, 3), 200, np.uint8)
    canvas, scale, left, top = yolo.letterbox(img, 640)
    assert canvas.shape == (640, 640, 3)
    assert scale == 1.0 and left == 0 and top == 80
    assert canvas[0, 0, 0] == yolo.PAD_VALUE and canvas[320, 320, 0] == 200


def test_preprocess_shape_and_range():
    img = np.random.randint(0, 255, (480, 640, 3), np.uint8)
    x, *_ = yolo.preprocess(img)
    assert x.shape == (1, 3, 640, 640) and x.dtype == np.float32
    assert 0.0 <= x.min() and x.max() <= 1.0


def test_decode_undoes_letterbox():
    # a box centred at original pixel (100, 50) of a 640x480 image sits at (100, 130) in the canvas
    dets = yolo.decode(fake_output([(100, 130, 40, 20, 2, 0.9)]), 1.0, 0, 80, 640, 480)
    assert len(dets) == 1
    d = dets[0]
    assert d.name == 'spill' and d.cls_id == 2
    assert (d.cx, d.cy, d.w, d.h) == pytest.approx((100, 50, 40, 20))


def test_decode_scaled_image():
    # 320x240 image letterboxed to 640: scale 2, top pad 80
    dets = yolo.decode(fake_output([(200, 280, 40, 40, 0, 0.8)]), 2.0, 0, 80, 320, 240)
    assert dets[0].cx == pytest.approx(100) and dets[0].cy == pytest.approx(100)
    assert dets[0].w == pytest.approx(20)


def test_decode_threshold_and_nms():
    out = fake_output([(300, 300, 50, 50, 0, 0.9), (305, 302, 50, 50, 0, 0.6),   # same patch twice
                       (500, 300, 50, 50, 0, 0.3), (100, 300, 50, 50, 3, 0.1)])  # weak, then too weak
    dets = yolo.decode(out, 1.0, 0, 80, 640, 480, conf_thr=0.25, iou_thr=0.5)
    assert [round(d.conf, 1) for d in dets] == [0.9, 0.3]


def test_nms_is_per_class():
    out = fake_output([(300, 300, 50, 50, 0, 0.9), (300, 300, 50, 50, 1, 0.8)])
    assert len(yolo.decode(out, 1.0, 0, 80, 640, 480)) == 2


def test_decode_empty_and_clipped():
    assert yolo.decode(fake_output([]), 1.0, 0, 80, 640, 480) == []
    d = yolo.decode(fake_output([(5, 90, 40, 40, 1, 0.9)]), 1.0, 0, 80, 640, 480)[0]
    assert d.x0 == 0.0 and d.y0 >= 0.0


MODEL = yolo.MODEL_RELATIVE


def _real_model():
    try:
        return yolo.find_model('')
    except FileNotFoundError:
        return None


@pytest.mark.skipif(_real_model() is None, reason='dirt_yolo11n.onnx not found')
def test_real_model_runs_and_reports_names():
    pytest.importorskip('onnxruntime')
    det = yolo.DirtDetector(_real_model())
    assert det.names == ('crumbs', 'dust', 'spill', 'hair') and det.size == 640
    out = det.detect(np.full((480, 640, 3), 128, np.uint8))     # plain grey floor: nothing to find
    assert isinstance(out, list) and len(out) <= 2


def test_find_model_explicit_missing_path_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        yolo.find_model(str(tmp_path / 'nope.onnx'))
